#include "rclcpp/rclcpp.hpp"
#include "pedsim_msgs/msg/agent_states.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <unordered_map>
#include <chrono>

using std::placeholders::_1;

class ObstacleProcessor : public rclcpp::Node
{
public:
    ObstacleProcessor()
        : Node("obstacle_ellipses"), obs_kf_buffer_size_(50)
    {
        RCLCPP_INFO(this->get_logger(), "obstacle_ellipses node has been started and is ready.");

        subscription_ = this->create_subscription<pedsim_msgs::msg::AgentStates>(
            "/pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleProcessor::agent_callback, this, _1));

        obstacle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("for_obs_track", 10);

        last_time_ = this->now();
    }

private:
    struct AgentState
    {
        float x, y;
        rclcpp::Time time;
    };

    void agent_callback(const pedsim_msgs::msg::AgentStates::SharedPtr msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0)
            dt = 0.1;

        std_msgs::msg::Float32MultiArray obstacle_msg;
        obstacle_msg.data.clear();

        for (const auto &agent : msg->agent_states)
        {
            float x = agent.pose.position.x;
            float y = agent.pose.position.y;

            tf2::Quaternion q(
                agent.pose.orientation.x,
                agent.pose.orientation.y,
                agent.pose.orientation.z,
                agent.pose.orientation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float a = 0.4f;
            float b = 0.2f;
            float theta = static_cast<float>(yaw);
            int label = static_cast<int>(agent.id);
            float mea_cov = 0.0f;

            // --- vx, vy ---
            float vx = 0.0f, vy = 0.0f;
            if (prev_agents_.find(agent.id) != prev_agents_.end())
            {
                const auto &prev = prev_agents_[agent.id];
                double delta_t = (current_time - prev.time).seconds();
                if (delta_t > 0.0)
                {
                    vx = (x - prev.x) / delta_t;
                    vy = (y - prev.y) / delta_t;
                }
            }

            prev_agents_[agent.id] = {x, y, current_time};

            obstacle_msg.data.push_back(x);
            obstacle_msg.data.push_back(y);
            obstacle_msg.data.push_back(vx);
            obstacle_msg.data.push_back(vy);
            obstacle_msg.data.push_back(a);
            obstacle_msg.data.push_back(b);
            obstacle_msg.data.push_back(theta);
            obstacle_msg.data.push_back(static_cast<float>(label));
            obstacle_msg.data.push_back(mea_cov);

            if (obstacle_buffer_.size() >= obs_kf_buffer_size_)
                obstacle_buffer_.erase(obstacle_buffer_.begin());
            obstacle_buffer_.push_back({x, y, vx, vy, a, b, theta});
        }

        obstacle_pub_->publish(obstacle_msg);
        last_time_ = current_time;
    }

    // ROS
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_pub_;

    std::unordered_map<int, AgentState> prev_agents_;
    std::vector<std::vector<float>> obstacle_buffer_;
    size_t obs_kf_buffer_size_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleProcessor>());
    rclcpp::shutdown();
    return 0;
}
