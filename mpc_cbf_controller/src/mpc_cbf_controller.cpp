#include "nav2_mpc_cbf_controller/mpc_cbf_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace nav2_mpc_cbf_controller
{

void MpcCbfPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node_shared = node_.lock();

  optimizer_client_ = node_shared->create_client<nav2_mpc_cbf_controller::srv::MpcCbfOptimize>("/mpc_cbf_optimize");

  obstacle_sub_ = node_shared->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/obs_predict_pub", rclcpp::QoS(10),
    std::bind(&MpcCbfPlanner::obstacleCallback, this, std::placeholders::_1));
}

void MpcCbfPlanner::cleanup() {}
void MpcCbfPlanner::activate() {}
void MpcCbfPlanner::deactivate() {}

void MpcCbfPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void MpcCbfPlanner::obstacleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  predicted_obstacles_.clear();
  
  const auto & data = msg->data;
  const size_t element_per_pose = 8;  // (x, y, vx, vy, a, b, theta, class_id)
  
  size_t num_poses = data.size() / element_per_pose;
  for (size_t i = 0; i < num_poses; ++i) {
    PredictedObstacle pred;
    pred.x = data[i * element_per_pose + 0];
    pred.y = data[i * element_per_pose + 1];
    // pred.vx = data[i * element_per_pose + 2];
    // pred.vy = data[i * element_per_pose + 3];
    pred.a = data[i * element_per_pose + 4];
    pred.b = data[i * element_per_pose + 5];
    pred.theta = data[i * element_per_pose + 6];
    pred.class_id = static_cast<int>(data[i * element_per_pose + 7]);
    
    predicted_obstacles_.push_back(pred);
  }
}

geometry_msgs::msg::TwistStamped MpcCbfPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node_shared = node_.lock();
  if (!optimizer_client_->wait_for_service(std::chrono::milliseconds(100))) {
    RCLCPP_WARN(node_shared->get_logger(), "Waiting for /mpc_cbf_optimize service...");
    throw std::runtime_error("Optimizer service not available");
  }

  auto request = std::make_shared<nav2_mpc_cbf_controller::srv::MpcCbfOptimize::Request>();
  request->current_pose = pose;
  request->goal_pose = global_plan_.poses.back();
  
  nav_msgs::msg::Path sampled_path;
  sampled_path.header = global_plan_.header;
  for (size_t i = 0; i < global_plan_.poses.size(); i += 5) {  // lấy mỗi 5 điểm
    sampled_path.poses.push_back(global_plan_.poses[i]);
  }
  request->reference_path = sampled_path;

  for (const auto & pred : predicted_obstacles_) {
    geometry_msgs::msg::Pose p;
    p.position.x = pred.x;
    p.position.y = pred.y;
    p.position.z = static_cast<double>(pred.class_id);
    p.orientation.x = pred.a;
    p.orientation.y = pred.b;
    p.orientation.z = pred.theta;
    p.orientation.w = 1.0;
    request->predicted_obstacles.poses.push_back(p);
  }

  auto result_future = optimizer_client_->async_send_request(request);

  if (result_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    throw std::runtime_error("Optimizer service not available");
  }

  auto response = result_future.get();

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_shared->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = response->v;
  cmd_vel.twist.angular.z = response->w;

  return cmd_vel;
}

}  // namespace nav2_mpc_cbf_controller

PLUGINLIB_EXPORT_CLASS(nav2_mpc_cbf_controller::MpcCbfPlanner, nav2_core::Controller)
