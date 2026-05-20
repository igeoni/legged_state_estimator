// Translation unit: pinocchio only — NO GTSAM headers.
#include "legged_state_estimator/fk_provider.hpp"
#include "legged_state_estimator/g1_kinematics.hpp"

namespace legged_state_estimator {

namespace {

class G1FkProvider final : public FkProvider {
 public:
  explicit G1FkProvider(const std::string& urdf_path)
      : kinematics_(urdf_path) {}

  Eigen::Vector3d footPosition(const Eigen::VectorXd& q_leg,
                               bool left) const override {
    return kinematics_.footPosition(q_leg, left);
  }

 private:
  G1Kinematics kinematics_;
};

}  // namespace

std::unique_ptr<FkProvider> makeG1FkProvider(const std::string& urdf_path) {
  return std::make_unique<G1FkProvider>(urdf_path);
}

}  // namespace legged_state_estimator
