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

#include <rclcpp/rclcpp.hpp>
#include "legged_state_estimator/visualizer/fk_visualizer.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<legged_state_estimator::FkVisualizer>());
  rclcpp::shutdown();
  return 0;
}
