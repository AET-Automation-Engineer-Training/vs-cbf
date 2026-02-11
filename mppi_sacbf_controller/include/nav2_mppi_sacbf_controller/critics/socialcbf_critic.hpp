// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_SACBF_CONTROLLER__CRITICS__SOCIALCBF_CRITIC_HPP_
#define NAV2_MPPI_SACBF_CONTROLLER__CRITICS__SOCIALCBF_CRITIC_HPP_

#include <vector>
#include <mutex>
#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_mppi_sacbf_controller/critic_function.hpp"
#include "nav2_mppi_sacbf_controller/models/state.hpp"
#include <eigen3/Eigen/Dense>

namespace mppi_sacbf::critics
{

enum PersonType { STRANGER = 0, FRIEND = 1, ELDER = 2 };

// Cập nhật cấu trúc SocialPrediction
struct SocialPrediction {
  float x, y;       // Position
  float a, b;       // Ellipse size
  float theta;      // Orientation
  float vx, vy;     // Velocity
  int class_id;     // Social category
};

class SocialCBFCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  float evaluateCBF(const geometry_msgs::msg::Pose2D & robot_pose, const SocialPrediction & person_pred);
  Eigen::Matrix2f constructRMatrix(float a, float b, float theta);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr social_zone_sub_;
  std::mutex data_mutex_;

  std::vector<SocialPrediction> people_predictions_;

  float social_radius_;
  float gamma_;
  float penalty_weight_;
  size_t sample_step_;

  std::unordered_map<int, float> class_radius_map_;
  std::unordered_map<int, float> class_penalty_weight_;
};

}  // namespace mppi_sacbf::critics

#endif  // NAV2_MPPI_SACBF_CONTROLLER__CRITICS__SOCIALCBF_CRITIC_HPP_