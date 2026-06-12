# legged_state_estimator

A ROS2 package for real-time state estimation of legged robots (humanoids). It fuses IMU measurements, joint kinematics, and foot contact forces using factor graph optimization (GTSAM Fixed-Lag Smoother) to estimate the robot's base pose, velocity, and IMU bias.


## Requirements

- Ubuntu 22.04
- ROS2 Humble

## Dependencies

### GTSAM (local install)

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select legged_state_estimator
source install/setup.bash
```

## Run

```bash
ros2 launch legged_state_estimator humanoid_state_estimator.launch.py
```

## Topics

### Input

| Topic | Type | Description |
|-------|------|-------------|
| `/imu` | `sensor_msgs/Imu` | IMU measurement |
| `/joint_states` | `sensor_msgs/JointState` | Joint states |
| `/contact/left_foot` | `geometry_msgs/WrenchStamped` | Left foot contact wrench |
| `/contact/right_foot` | `geometry_msgs/WrenchStamped` | Right foot contact wrench |
| `/description` | `std_msgs/String` | Robot URDF description |

### Output

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimator/odom` | `nav_msgs/Odometry` | Estimated base odometry |
| `/state_estimator/path` | `nav_msgs/Path` | Estimated base path |
| `/state_estimator/foot_contacts` | `ContactStateArray` | Foot contact states |

## License

BSD-3-Clause. See [LICENSE](LICENSE) for details.
This work builds upon [GTSAM](https://github.com/borglab/gtsam), distributed under its own BSD license.
