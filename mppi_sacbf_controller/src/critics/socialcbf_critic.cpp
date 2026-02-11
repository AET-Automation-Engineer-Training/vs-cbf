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

#include "nav2_mppi_sacbf_controller/critics/socialcbf_critic.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace mppi_sacbf::critics
{

void SocialCBFCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(gamma_, "gamma", 0.5f);                 // gamma: CBF penalty sharpness
  getParam(penalty_weight_, "penalty_weight", 10.0f);  // global scaling of social cost
  getParam(sample_step_, "sample_step", 25);       // prediction horizon T

  auto node = parent_.lock();
  social_zone_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/obs_predict_pub", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      people_predictions_.clear();
      const auto & data = msg->data;
      const size_t num_blocks = data.size() / 8;

      for (size_t i = 0; i < num_blocks; ++i) {
        SocialPrediction pred;

        pred.x = data[8 * i + 0];
        pred.y = data[8 * i + 1];
        pred.vx = data[8 * i + 2];                    // directly assigned from prediction
        pred.vy = data[8 * i + 3];
        pred.a = data[8 * i + 4];
        pred.b = data[8 * i + 5];
        pred.theta = data[8 * i + 6];                 // retained original orientation
        pred.class_id = static_cast<int>(data[8 * i + 7]);

        people_predictions_.push_back(pred);
      }
    });

  class_penalty_weight_ = {
    {STRANGER, 1.5f},
    {FRIEND, 0.8f},
    {ELDER, 2.0f}
  };

  RCLCPP_INFO(logger_, "SocialCBFCritic initialized with %zu prediction steps.", sample_step_);
}

void SocialCBFCritic::score(CriticData & data)
{
  if (!enabled_) return;

  const size_t num_trajs = data.trajectories.x.shape(0);
  const size_t traj_len = data.trajectories.x.shape(1);

  std::lock_guard<std::mutex> lock(data_mutex_);
  xt::xtensor<float, 1> cbf_cost = xt::zeros<float>({num_trajs});
  const size_t num_people = people_predictions_.size() / sample_step_;

  for (size_t i = 0; i < num_trajs; ++i) {
    float penalty = 0.0f;

    for (size_t k = 0; k < traj_len && k < sample_step_; ++k) {
      geometry_msgs::msg::Pose2D pose;
      pose.x = data.trajectories.x(i, k);
      pose.y = data.trajectories.y(i, k);
      pose.theta = data.trajectories.yaws(i, k);

      for (size_t j = 0; j < num_people; ++j) {
        const size_t idx = j * sample_step_ + k;
        if (idx >= people_predictions_.size()) continue;

        const auto & pred = people_predictions_[idx];

        // Compute Control Barrier Function:
        // h(x_k) = 1 - (p(x_k) - x_h(k))^T R^{-1} (p(x_k) - x_h(k))
        float h = evaluateCBF(pose, pred);

        if (h > 0.0f) {
          // Directional weight based on cos(α) = (v_h · d) / (||v_h|| · ||d||)
          Eigen::Vector2f dir(pred.vx, pred.vy);
          Eigen::Vector2f delta(pose.x - pred.x, pose.y - pred.y);
          float cos_theta = dir.norm() > 0 && delta.norm() > 0 ?
            dir.normalized().dot(delta.normalized()) : 0.0f;

          // Apply class-specific weighted penalty:
          // φ(h, α) = λ_c (1 + max(0, cos(α))) exp(-γ h)
          float weight = class_penalty_weight_[pred.class_id] * (1.0f + std::max(0.0f, cos_theta));
          penalty += weight * std::exp(-gamma_ * h);
        }
      }
    }

    cbf_cost[i] = penalty;
  }

  // Add total CBF-based social cost to MPPI cost
  data.costs += penalty_weight_ * cbf_cost;
}

// CBF evaluation: h = 1 - d^T R^{-1} d
float SocialCBFCritic::evaluateCBF(const geometry_msgs::msg::Pose2D & robot_pose, const SocialPrediction & pred)
{
  Eigen::Vector2f d(robot_pose.x - pred.x, robot_pose.y - pred.y);
  Eigen::Matrix2f R = constructRMatrix(pred.a, pred.b, pred.theta);
  float value = d.transpose() * R.inverse() * d;
  return 1.0f - value;
}

// Construct ellipse metric matrix R = R(θ) D R(θ)^T with D = diag(a^2, b^2)
Eigen::Matrix2f SocialCBFCritic::constructRMatrix(float a, float b, float theta)
{
  float c = std::cos(theta);
  float s = std::sin(theta);
  Eigen::Matrix2f rot;
  rot << c, -s, s, c;

  Eigen::Matrix2f D;
  D << a * a, 0, 0, b * b;

  return rot * D * rot.transpose();
}

}  // namespace mppi_sacbf::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mppi_sacbf::critics::SocialCBFCritic, mppi_sacbf::critics::CriticFunction)