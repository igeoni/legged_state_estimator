/*
 * Copyright (c) 2026, Geoni Lee, LAIR Lab, Sungkyunkwan University (SKKU)
 * All Rights Reserved
 *
 * This file is licensed under the BSD-3-Clause License.
 * This work builds upon GTSAM (Georgia Tech Smoothing and Mapping library),
 * developed at Georgia Tech Research Corporation. GTSAM is distributed under
 * its own BSD license; see https://github.com/borglab/gtsam for details.
 *
 * See LICENSE for the license information applicable to this file.
 */

#include "legged_state_estimator/kinematics/fk_provider.hpp"
#include "legged_state_estimator/kinematics/g1_kinematics.hpp"

namespace legged_state_estimator {

namespace {

class G1FkProvider final : public FkProvider {
 public:
  explicit G1FkProvider(const std::string& urdf_path) : kinematics_(urdf_path) {}

  Eigen::Vector3d footPosition(const Eigen::VectorXd& q_leg, bool left) const override {
    return kinematics_.footPosition(q_leg, left);
  }

 private:
  G1Kinematics kinematics_;
};

}  // namespace

std::unique_ptr<FkProvider> makeG1FkProvider(const std::string& urdf_path) {
  return std::make_unique<G1FkProvider>(urdf_path);
}

}  
