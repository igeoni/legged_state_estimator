#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace legged_state_estimator {

/**
 * Accumulates (feature_vector, contact_label) samples and persists them
 * as train / val / test .npy splits.
 *
 * Feature layout (42 floats):
 *   q[12]     joint angles        (6 joints × 2 legs)
 *   qd[12]    joint velocities    (6 joints × 2 legs)
 *   acc[3]    IMU linear acceleration
 *   omega[3]  IMU angular velocity
 *   p[6]      FK foot positions   (left_foot xyz, right_foot xyz)
 *   v[6]      FK foot velocities  (left_foot xyz, right_foot xyz)
 *
 * Label encoding (bipedal, int32):
 *   0 = no contact
 *   1 = right foot only
 *   2 = left foot only
 *   3 = both feet
 */
class DataBuffer {
 public:
  static constexpr int kFeatureDim = 42;

  struct SplitRatio {
    double train = 0.70;
    double val   = 0.15;
    // test = 1 - train - val
  };

  explicit DataBuffer(const std::string& output_dir);
  DataBuffer(const std::string& output_dir, const SplitRatio& ratio);

  void addSample(const std::vector<float>& feature, int32_t label);

  std::size_t size() const { return labels_.size(); }

  // Shuffle and write train/val/test .npy files to output_dir_.
  void save() const;

 private:
  std::string output_dir_;
  SplitRatio ratio_;

  std::vector<float>   features_;   // row-major: size() × kFeatureDim
  std::vector<int32_t> labels_;

  void saveSplit(const std::string& prefix,
                 const std::vector<std::size_t>& indices) const;
};

}  // namespace legged_state_estimator
