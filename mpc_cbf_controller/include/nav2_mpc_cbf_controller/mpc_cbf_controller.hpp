#ifndef NAV2_MPC_CBF_CONTROLLER__MPC_CBF_PLANNER_HPP_
#define NAV2_MPC_CBF_CONTROLLER__MPC_CBF_PLANNER_HPP_

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <nav2_mpc_cbf_controller/srv/mpc_cbf_optimize.hpp>

namespace nav2_mpc_cbf_controller
{

struct PredictedObstacle
{
  double x;
  double y;
  double a;
  double b;
  double theta;
  int class_id;
};

class MpcCbfPlanner : public nav2_core::Controller
{
public:
  MpcCbfPlanner() = default;
  ~MpcCbfPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {}

private:
  void obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;

  rclcpp::Client<nav2_mpc_cbf_controller::srv::MpcCbfOptimize>::SharedPtr optimizer_client_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;

  std::vector<PredictedObstacle> predicted_obstacles_;
};

}  // namespace nav2_mpc_cbf_controller

#endif  // NAV2_MPC_CBF_CONTROLLER__MPC_CBF_PLANNER_HPP_
