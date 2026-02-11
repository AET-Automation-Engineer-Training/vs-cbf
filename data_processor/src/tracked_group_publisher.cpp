#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "pedsim_msgs/msg/agent_groups.hpp"
#include "pedsim_msgs/msg/agent_group.hpp"
#include "pedsim_msgs/msg/tracked_groups.hpp"
#include "pedsim_msgs/msg/tracked_group.hpp"
#include "builtin_interfaces/msg/duration.hpp"

class GroupConverterNode : public rclcpp::Node
{
public:
  GroupConverterNode()
    : Node("group_converter_node")
  {
    sub_ = this->create_subscription<pedsim_msgs::msg::AgentGroups>(
      "/pedsim_simulator/simulated_groups", 10,
      std::bind(&GroupConverterNode::groupCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<pedsim_msgs::msg::TrackedGroups>(
      "/tracked_groups", 10);
  }

private:
  void groupCallback(const pedsim_msgs::msg::AgentGroups::SharedPtr msg)
  {
    pedsim_msgs::msg::TrackedGroups tracked_groups_msg;
    tracked_groups_msg.header = msg->header;

    for (const auto& agent_group : msg->groups)
    {
      pedsim_msgs::msg::TrackedGroup tracked_group;
      tracked_group.group_id = static_cast<uint64_t>(agent_group.group_id);

      // Convert float64 age -> builtin_interfaces/Duration
      tracked_group.age.sec = static_cast<int32_t>(agent_group.age);
      tracked_group.age.nanosec = static_cast<uint32_t>((agent_group.age - tracked_group.age.sec) * 1e9);

      // Pose -> PoseWithCovariance (giữ nguyên pose, giả định covariance đơn giản)
      tracked_group.center_of_gravity.pose = agent_group.center_of_mass;
      for (int i = 0; i < 36; ++i)
      {
        tracked_group.center_of_gravity.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
      }

      tracked_group.track_ids = agent_group.members;

      tracked_groups_msg.groups.push_back(tracked_group);
    }

    pub_->publish(tracked_groups_msg);
  }

  rclcpp::Subscription<pedsim_msgs::msg::AgentGroups>::SharedPtr sub_;
  rclcpp::Publisher<pedsim_msgs::msg::TrackedGroups>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroupConverterNode>());
  rclcpp::shutdown();
  return 0;
}
