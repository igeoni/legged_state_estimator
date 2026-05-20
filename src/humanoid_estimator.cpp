#include "legged_state_estimator/humanoid_estimator.hpp"

#include <stdexcept>

namespace legged_state_estimator {

HumanoidEstimator::HumanoidEstimator(const HumanoidEstimatorParams& params)
    : params_(params) {}

gtsam::LeggedEstimatorParams HumanoidEstimator::makeGtsamParams(
    const gtsam::imuBias::ConstantBias& bias) const {
  auto preint = std::make_shared<gtsam::PreintegrationParams>(params_.gravity);
  preint->gyroscopeCovariance =
      gtsam::Matrix3::Identity() * (params_.sigma_gyro * params_.sigma_gyro);
  preint->accelerometerCovariance =
      gtsam::Matrix3::Identity() * (params_.sigma_acc * params_.sigma_acc);
  preint->integrationCovariance =
      gtsam::Matrix3::Identity() *
      (params_.sigma_integration * params_.sigma_integration);

  gtsam::LeggedEstimatorParams p;
  p.preintegrationParams = preint;
  p.body_P_imu = params_.body_P_imu;
  p.imuBias = bias;
  p.biasAccRandomWalkSigma = params_.bias_acc_rw;
  p.biasOmegaRandomWalkSigma = params_.bias_omega_rw;
  p.contactCovariance =
      gtsam::Vector3(params_.contact_sigma_xy * params_.contact_sigma_xy,
                     params_.contact_sigma_xy * params_.contact_sigma_xy,
                     params_.contact_sigma_z * params_.contact_sigma_z)
          .asDiagonal();
  p.useFullContactInitialization = false;  // initial attitude computed from IMU gravity
  return p;
}

void HumanoidEstimator::initialize(
    const gtsam::imuBias::ConstantBias& bias,
    const gtsam::Rot3& initial_attitude) {
  const size_t num_feet = params_.foot_names.size();

  const gtsam::NavState initial_state(
      initial_attitude,
      gtsam::Point3(0.0, 0.0, params_.initial_height),
      gtsam::Vector3::Zero());

  // Zero foothold initialization
  gtsam::Matrix footholds =
      gtsam::Matrix::Zero(3, static_cast<int>(num_feet));

  const gtsam::LeggedEstimatorParams gtsam_params = makeGtsamParams(bias);

  const int dim = 9 + 3 * static_cast<int>(num_feet);
  gtsam::Matrix covariance = gtsam::Matrix::Zero(dim, dim);
  covariance.diagonal().head(9) =
      (gtsam::Vector(9) << 1e-2, 1e-2, 1e-6, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05)
          .finished();
  covariance.diagonal().tail(3 * static_cast<int>(num_feet)).setConstant(
      gtsam_params.footholdInitSigma * gtsam_params.footholdInitSigma);

  gtsam::Matrix9 base_cov = gtsam::Matrix9::Zero();
  base_cov.diagonal() = covariance.diagonal().head(9);

  const std::string& type = params_.estimator_type;
  if (type == "invariant_ekf") {
    estimator_ = std::make_unique<gtsam::LeggedInvariantEKF>(
        initial_state, footholds, covariance, gtsam_params, params_.foot_names);
  } else if (type == "invariant_graph") {
    estimator_ = std::make_unique<gtsam::LeggedInvariantIEKF>(
        initial_state, footholds, covariance, gtsam_params, params_.foot_names);
  } else if (type == "fixed_lag_single_bias") {
    estimator_ = std::make_unique<gtsam::LeggedFixedLagSmoother>(
        initial_state, footholds, base_cov, gtsam_params, params_.lag_seconds,
        params_.foot_names);
  } else if (type == "fixed_lag_combined_bias") {
    estimator_ = std::make_unique<gtsam::LeggedCombinedFixedLagSmoother>(
        initial_state, footholds, base_cov, gtsam_params, params_.lag_seconds,
        params_.foot_names);
  } else {
    throw std::runtime_error("Unknown estimator_type: " + type);
  }
}

void HumanoidEstimator::predict(const gtsam::Vector3& omega,
                                const gtsam::Vector3& accel, double dt) {
  if (!estimator_) return;
  estimator_->predict(omega, accel, dt);
}

void HumanoidEstimator::processContacts(
    const std::vector<gtsam::ContactMeasurement>& active_contacts) {
  if (!estimator_) return;
  estimator_->processContacts(active_contacts);
}

gtsam::NavState HumanoidEstimator::navState() const {
  if (!estimator_) return {};
  const auto est = estimator_->estimate();
  // Layout: (R, p, v, f_1, ..., f_k) — x(0)=position, x(1)=velocity
  return gtsam::NavState(est.rotation(), est.x(0), est.x(1));
}

gtsam::imuBias::ConstantBias HumanoidEstimator::bias() const {
  if (!estimator_) return {};
  return estimator_->estimateBias();
}

gtsam::Rot3 HumanoidEstimator::orientation() const {
  return navState().attitude();
}

gtsam::Point3 HumanoidEstimator::position() const {
  return navState().position();
}

gtsam::Vector3 HumanoidEstimator::velocity() const {
  return navState().velocity();
}

}  // namespace legged_state_estimator
