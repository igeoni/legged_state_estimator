#!/usr/bin/env python3

import collections
import os

import numpy as np
import torch
import torch.nn as nn
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int32
from geometry_msgs.msg import WrenchStamped

WINDOW_SIZE = 150
FEATURE_DIM = 42

LEG_JOINT_NAMES = [
    "left_hip_pitch_joint",  "left_hip_roll_joint",  "left_hip_yaw_joint",
    "left_knee_joint",       "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
]


# ---------------------------------------------------------------------------
# Model (must match training code exactly)
# ---------------------------------------------------------------------------

class ContactCnnG1(nn.Module):
    def __init__(self):
        super().__init__()
        self.block1 = nn.Sequential(
            nn.Conv1d(42, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv1d(64, 64, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.MaxPool1d(kernel_size=2, stride=2),
        )
        self.block2 = nn.Sequential(
            nn.Conv1d(64, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv1d(128, 128, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.MaxPool1d(kernel_size=2, stride=2),
        )
        # window_size=150 → pool×2 → 37, channels=128 → 128×37=4736
        self.fc = nn.Sequential(
            nn.Linear(4736, 2048),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(2048, 512),
            nn.ReLU(),
            nn.Dropout(p=0.5),
            nn.Linear(512, 4),
        )

    def forward(self, x):
        x = x.permute(0, 2, 1)   # (B, 150, 42) → (B, 42, 150)
        x = self.block1(x)        # → (B, 64, 75)
        x = self.block2(x)        # → (B, 128, 37)
        x = x.view(x.shape[0], -1)
        return self.fc(x)         # → (B, 4)


# ---------------------------------------------------------------------------
# Label encoding:  0=no contact, 1=right only, 2=left only, 3=both
# ---------------------------------------------------------------------------

def decode_label(label: int):
    left  = bool(label & 2)
    right = bool(label & 1)
    return left, right


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class ContactEstimatorNode(Node):

    def __init__(self):
        super().__init__('contact_estimator')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('window_size', WINDOW_SIZE)
        self.declare_parameter('time_slop_ns', 50_000_000)
        self.declare_parameter('topic.imu',           '/imu')
        self.declare_parameter('topic.joint_states',  '/joint_states')
        self.declare_parameter('topic.contact_label', '/contact_estimator/label')
        self.declare_parameter('topic.left_contact',  '/contact_estimator/left_foot')
        self.declare_parameter('topic.right_contact', '/contact_estimator/right_foot')

        model_path   = self.get_parameter('model_path').value
        urdf_path    = self.get_parameter('urdf_path').value
        window_size  = self.get_parameter('window_size').value
        self._slop   = self.get_parameter('time_slop_ns').value

        if not model_path:
            raise RuntimeError("Parameter 'model_path' is required but not set.")
        if not urdf_path:
            raise RuntimeError("Parameter 'urdf_path' is required but not set.")

        # --- Load model ---
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {self._device}')

        self._model = ContactCnnG1().to(self._device)
        ckpt = torch.load(model_path, map_location=self._device)
        self._model.load_state_dict(ckpt['model_state_dict'])
        self._model.eval()
        self.get_logger().info(f'Model loaded from {model_path}  'f'(epoch={ckpt["epoch"]}, val_acc={ckpt["val_acc"]:.4f})')

        # --- Pinocchio kinematics ---
        self.get_logger().info(f'Loading URDF from {urdf_path}')
        self._pin_model = pin.buildModelFromUrdf(urdf_path)
        self._pin_data  = self._pin_model.createData()
        self._init_kinematics()

        # --- Sliding window ---
        self._window_size = window_size
        self._window = collections.deque(maxlen=window_size)

        # --- Sync state ---
        self._latest_imu   = None
        self._latest_joint = None
        self._joint_map: dict[str, int] = {}
        self._joint_map_ready = False

        # --- Subscribers ---
        self.create_subscription(Imu, self.get_parameter('topic.imu').value, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(JointState, self.get_parameter('topic.joint_states').value, self._joint_cb, qos_profile_sensor_data)

        # --- Publishers ---
        self._label_pub = self.create_publisher(Int32,         self.get_parameter('topic.contact_label').value, 10)
        self._left_pub  = self.create_publisher(WrenchStamped, self.get_parameter('topic.left_contact').value,  10)
        self._right_pub = self.create_publisher(WrenchStamped, self.get_parameter('topic.right_contact').value, 10)

        self.get_logger().info(
            f'ContactEstimatorNode ready  '
            f'(window={self._window_size}, slop={self._slop/1e6:.0f} ms)'
        )

    # -----------------------------------------------------------------------
    # Pinocchio helpers
    # -----------------------------------------------------------------------

    def _init_kinematics(self):
        left_names  = LEG_JOINT_NAMES[:6]
        right_names = LEG_JOINT_NAMES[6:]

        def q_v_idx(names):
            q_idx, v_idx = [], []
            for n in names:
                jid = self._pin_model.getJointId(n)
                q_idx.append(self._pin_model.joints[jid].idx_q)
                v_idx.append(self._pin_model.joints[jid].idx_v)
            return q_idx, v_idx

        self._left_q_idx,  self._left_v_idx  = q_v_idx(left_names)
        self._right_q_idx, self._right_v_idx = q_v_idx(right_names)

        self._left_foot_id  = self._pin_model.getFrameId('left_ankle_roll_link')
        self._right_foot_id = self._pin_model.getFrameId('right_ankle_roll_link')
        self.get_logger().info('Pinocchio kinematics initialized')

    def _foot_position(self, q_leg: np.ndarray, left: bool) -> np.ndarray:
        q = pin.neutral(self._pin_model)
        idx = self._left_q_idx if left else self._right_q_idx
        for i in range(6):
            q[idx[i]] = q_leg[i]
        pin.forwardKinematics(self._pin_model, self._pin_data, q)
        pin.updateFramePlacements(self._pin_model, self._pin_data)
        fid = self._left_foot_id if left else self._right_foot_id
        return self._pin_data.oMf[fid].translation.copy()

    def _foot_velocity(self, q_leg: np.ndarray, qd_leg: np.ndarray, left: bool) -> np.ndarray:
        q = pin.neutral(self._pin_model)
        v = np.zeros(self._pin_model.nv)
        q_idx = self._left_q_idx if left else self._right_q_idx
        v_idx = self._left_v_idx if left else self._right_v_idx
        for i in range(6):
            q[q_idx[i]] = q_leg[i]
            v[v_idx[i]] = qd_leg[i]
        pin.forwardKinematics(self._pin_model, self._pin_data, q, v)
        pin.updateFramePlacements(self._pin_model, self._pin_data)
        fid = self._left_foot_id if left else self._right_foot_id
        vel = pin.getFrameVelocity(
            self._pin_model, self._pin_data, fid, pin.LOCAL_WORLD_ALIGNED)
        return vel.linear.copy()

    # -----------------------------------------------------------------------
    # ROS2 callbacks
    # -----------------------------------------------------------------------

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _imu_cb(self, msg: Imu):
        self._latest_imu = msg
        self._try_process()

    def _joint_cb(self, msg: JointState):
        self._latest_joint = msg
        self._try_process()

    def _try_process(self):
        if self._latest_imu is None or self._latest_joint is None:
            return

        t_imu   = self._stamp_ns(self._latest_imu.header.stamp)
        t_joint = self._stamp_ns(self._latest_joint.header.stamp)
        if abs(t_imu - t_joint) > self._slop:
            return

        if not self._joint_map_ready:
            self._build_joint_map(self._latest_joint)
            if not self._joint_map_ready:
                return

        q_all, qd_all = self._extract_leg_joints(self._latest_joint)
        if q_all is None:
            return

        # FK
        p_left  = self._foot_position(q_all[:6],  True)
        p_right = self._foot_position(q_all[6:],  False)
        v_left  = self._foot_velocity(q_all[:6],  qd_all[:6],  True)
        v_right = self._foot_velocity(q_all[6:],  qd_all[6:],  False)

        # 42-dim feature vector  (identical to ContactDataCollector)
        imu = self._latest_imu
        feat = np.concatenate([
            q_all, qd_all,
            [imu.linear_acceleration.x,
             imu.linear_acceleration.y,
             imu.linear_acceleration.z,
             imu.angular_velocity.x,
             imu.angular_velocity.y,
             imu.angular_velocity.z],
            p_left, p_right, v_left, v_right,
        ]).astype(np.float32)

        self._latest_imu   = None
        self._latest_joint = None

        self._window.append(feat)
        if len(self._window) < self._window_size:
            return

        # --- Inference ---
        arr  = np.array(self._window, dtype=np.float32)          # (150, 42)
        mean = arr.mean(axis=0)
        std  = arr.std(axis=0)
        std[std < 1e-8] = 1.0                                    # avoid div-by-zero for constant features
        arr  = (arr - mean) / std
        x    = torch.from_numpy(arr).unsqueeze(0).to(self._device)  # (1, 150, 42)

        with torch.no_grad():
            logits = self._model(x)
            label  = int(torch.argmax(logits, dim=1).item())

        left_contact, right_contact = decode_label(label)

        stamp = self.get_clock().now().to_msg()

        label_msg      = Int32()
        label_msg.data = label

        left_msg                    = WrenchStamped()
        left_msg.header.stamp       = stamp
        left_msg.header.frame_id    = 'left_ankle_roll_link'
        left_msg.wrench.force.z     = 1.0 if left_contact  else 0.0

        right_msg                   = WrenchStamped()
        right_msg.header.stamp      = stamp
        right_msg.header.frame_id   = 'right_ankle_roll_link'
        right_msg.wrench.force.z    = 1.0 if right_contact else 0.0

        self._label_pub.publish(label_msg)
        self._left_pub.publish(left_msg)
        self._right_pub.publish(right_msg)

    def _build_joint_map(self, msg: JointState):
        self._joint_map = {name: i for i, name in enumerate(msg.name)}
        missing = [n for n in LEG_JOINT_NAMES if n not in self._joint_map]
        if missing:
            self.get_logger().error(
                f'Missing joints in /joint_states: {missing}', throttle_duration_sec=5.0)
            return
        self._joint_map_ready = True
        self.get_logger().info('Joint index map ready')

    def _extract_leg_joints(self, msg: JointState):
        q  = np.zeros(12, dtype=np.float64)
        qd = np.zeros(12, dtype=np.float64)
        for i, name in enumerate(LEG_JOINT_NAMES):
            idx = self._joint_map.get(name)
            if idx is None:
                return None, None
            q[i]  = msg.position[idx] if idx < len(msg.position) else 0.0
            qd[i] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0
        return q, qd


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ContactEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
