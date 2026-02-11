#include "rclcpp/rclcpp.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "pedsim_msgs/msg/tracked_persons.hpp"
#include "pedsim_msgs/msg/tracked_person.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "builtin_interfaces/msg/duration.hpp"

using std::placeholders::_1;

class TrackedPersonPublisher : public rclcpp::Node
{
public:
    TrackedPersonPublisher()
        : Node("tracked_person_publisher")
    {
        // Declare parameters for offset
        this->declare_parameter("offset_x", -7.0);
        this->declare_parameter("offset_y", -7.0);

        this->get_parameter("offset_x", offset_x_);
        this->get_parameter("offset_y", offset_y_);

        // Subscribe to agent states
        agent_sub_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
            "/pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
            std::bind(&TrackedPersonPublisher::agent_callback, this, _1));

        // Publisher for tracked persons
        tracked_pub_ = this->create_publisher<pedsim_msgs::msg::TrackedPersons>(
            "/tracked_persons", 10);
    }

private:
    void agent_callback(const pedsim_msgs::msg::AgentStates::SharedPtr msg)
    {
        // Re-read parameters in case they were dynamically changed
        this->get_parameter("offset_x", offset_x_);
        this->get_parameter("offset_y", offset_y_);

        pedsim_msgs::msg::TrackedPersons tracked_msg;
        tracked_msg.header.stamp = this->now();
        tracked_msg.header.frame_id = "map";

        for (const auto& agent : msg->agent_states)
        {
            pedsim_msgs::msg::TrackedPerson person;

            person.track_id = static_cast<uint64_t>(agent.id);
            person.detection_id = static_cast<uint64_t>(agent.id);
            person.is_occluded = false;
            person.is_matched = true;

            // Initialize age to zero
            builtin_interfaces::msg::Duration age;
            age.sec = 0;
            age.nanosec = 0;
            person.age = age;

            // Create pose with covariance, subtract offset
            geometry_msgs::msg::PoseWithCovariance pose_cov(
                rosidl_runtime_cpp::MessageInitialization::ALL);
            pose_cov.pose = agent.pose;
            pose_cov.pose.position.x -= offset_x_;
            pose_cov.pose.position.y -= offset_y_;

            // Set covariance: trust x, y; unknown z, yaw
            for (int i = 0; i < 36; ++i)
                pose_cov.covariance[i] = 0.0;
            pose_cov.covariance[0] = 0.1;      // x
            pose_cov.covariance[7] = 0.1;      // y
            pose_cov.covariance[14] = 999999.0; // z unknown
            pose_cov.covariance[35] = 999999.0; // yaw unknown

            person.pose = pose_cov;

            // Twist with covariance
            geometry_msgs::msg::TwistWithCovariance twist_cov(
                rosidl_runtime_cpp::MessageInitialization::ALL);
            twist_cov.twist = agent.twist;

            for (int i = 0; i < 36; ++i)
                twist_cov.covariance[i] = 0.0;
            twist_cov.covariance[0] = 0.1;   // vx
            twist_cov.covariance[7] = 0.1;   // vy
            twist_cov.covariance[14] = 999999.0;
            twist_cov.covariance[35] = 999999.0;

            person.twist = twist_cov;

            tracked_msg.tracks.push_back(person);
        }

        tracked_pub_->publish(tracked_msg);
    }

    double offset_x_;
    double offset_y_;

    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_sub_;
    rclcpp::Publisher<pedsim_msgs::msg::TrackedPersons>::SharedPtr tracked_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedPersonPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
