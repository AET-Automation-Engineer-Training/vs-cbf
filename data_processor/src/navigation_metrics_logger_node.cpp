#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>
#include <map>
#include <vector>
#include <utility>
#include <filesystem>
#include <chrono>
#include <cstdlib>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigationMetricsLogger : public rclcpp::Node {
public:
    NavigationMetricsLogger()
        : Node("navigation_metrics_logger"),
          robot_x_(0.0), robot_y_(0.0),
          path_length_(0.0), min_distance_(std::numeric_limits<double>::infinity()),
          tracking_(false), first_point_(true), idle_counter_(0),
          cmd_active_(false), goal_sent_(false), session_id_(0)
    {
        // Declare parameters
        this->declare_parameter("fixed_goal_x", 1.0);
        this->declare_parameter("fixed_goal_y", 5.0);
        this->declare_parameter("comfort_radius", 1.0);
        this->get_parameter("fixed_goal_x", fixed_goal_x_);
        this->get_parameter("fixed_goal_y", fixed_goal_y_);

        // Create log directory
        const char* home = std::getenv("HOME");
        log_dir_ = home ? std::string(home) + "/nav_logs/" : "/tmp/nav_logs/";
        std::filesystem::create_directories(log_dir_);

        pos_csv_filename_ = log_dir_ + "robot_positions_log.csv";
        nav_csv_filename_ = log_dir_ + "navigation_full_log.csv";
        session_file_ = log_dir_ + ".nav_logger_session_id";

        init_csv_headers();
        load_session_id();

        // Subscriptions
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationMetricsLogger::odom_callback, this, std::placeholders::_1));

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&NavigationMetricsLogger::cmd_callback, this, std::placeholders::_1));

        agents_sub_ = create_subscription<pedsim_msgs::msg::AgentStates>(
            "/pedsim_simulator/simulated_agents", 10,
            std::bind(&NavigationMetricsLogger::agents_callback, this, std::placeholders::_1));

        ellipses_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/obs_predict_pub", 10, std::bind(&NavigationMetricsLogger::ellipses_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(100ms, std::bind(&NavigationMetricsLogger::timer_callback, this));

        // Action client
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Gửi goal 1 lần khi Nav2 sẵn sàng
        goal_timer_ = this->create_wall_timer(1s, [this]() {
            if (goal_sent_) {
                goal_timer_->cancel();
                return;
            }

            if (!nav_action_client_->action_server_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Nav2 action server not ready yet, retrying...");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Nav2 action server is READY. Sending goal now!");
            start_new_session();
            send_nav_goal(fixed_goal_x_, fixed_goal_y_);
            goal_timer_->cancel();
        });
    }

    void on_shutdown() {
        if (tracking_) {
            RCLCPP_WARN(this->get_logger(),
                "Node shutting down — saving session as 'Canceled'");

            double total_time = (now() - start_time_).seconds();

            write_all_to_csv(path_length_, min_distance_, total_time, "Canceled");
            write_positions_to_csv();

            RCLCPP_WARN(this->get_logger(),
                "=== END SESSION %d (Canceled) — saved ===", session_id_);
        }
    }

private:
    // --- Robot state ---
    double robot_x_, robot_y_;
    double fixed_goal_x_, fixed_goal_y_;
    double path_length_, min_distance_;
    bool tracking_, first_point_;
    double last_x_, last_y_;
    rclcpp::Time start_time_, last_time_;

    // --- ROS objects ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agents_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ellipses_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr goal_timer_;
    std::weak_ptr<GoalHandleNavigateToPose> nav_to_pose_goal_handle_;

    // --- Logging ---
    std::string log_dir_, pos_csv_filename_, nav_csv_filename_, session_file_;
    int session_id_;
    std::vector<std::pair<double, double>> session_positions_;
    std::map<uint64_t, double> min_distances_per_agent_;
    std::map<uint64_t, double> violation_time_per_agent_;

    // --- Motion flags ---
    bool cmd_active_;
    rclcpp::Time last_cmd_time_;
    bool goal_sent_;
    int idle_counter_;
    const int idle_threshold_ = 50;
    const double vel_threshold_ = 0.05;
    const double ang_threshold_ = 0.05;
    const double cmd_timeout_ = 1.0;


    // --- Ellipses ---
    std::vector<double> ellipses_data_;
    std::vector<std::pair<uint64_t, std::pair<double, double>>> agent_positions_;

    // ============================================================
    //                        METHODS
    // ============================================================

    void send_nav_goal(double x, double y)
    {
        using NavigateToPose = nav2_msgs::action::NavigateToPose;

        if (goal_sent_) {
            RCLCPP_INFO(this->get_logger(), "Goal already sent, skipping...");
            return;
        }

        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(), "Nav2 action server not available.");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationMetricsLogger::handle_nav_result, this, std::placeholders::_1);

        auto goal_future = nav_action_client_->async_send_goal(goal, send_goal_options);

        std::thread([this, goal_future]() mutable {
            auto future_status = goal_future.wait_for(std::chrono::seconds(4));
            if (future_status == std::future_status::ready) {
                try {
                    auto goal_handle = goal_future.get();
                    if (!goal_handle) {
                        RCLCPP_WARN(this->get_logger(), "Goal was rejected by Nav2.");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2!");
                        this->goal_sent_ = true;
                        this->nav_to_pose_goal_handle_ = goal_handle;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while sending goal: %s", e.what());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Timed out waiting for Nav2 goal response.");
            }
        }).detach();

        RCLCPP_INFO(this->get_logger(), "Sent goal to Nav2: (%.2f, %.2f)", x, y);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (std::fabs(msg->linear.x) > 1e-3 || std::fabs(msg->angular.z) > 1e-3) {
            cmd_active_ = true;
            last_cmd_time_ = now();
        } else {
            cmd_active_ = false;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double translational_speed = std::hypot(vx, vy);
        double rotational_speed = std::fabs(msg->twist.twist.angular.z);

        bool moving = (translational_speed > vel_threshold_ || rotational_speed > ang_threshold_);
        bool recent_cmd = cmd_active_ && (now() - last_cmd_time_).seconds() < cmd_timeout_;

        if (tracking_) {
            if (!moving) idle_counter_++; else idle_counter_ = 0;

            if (first_point_) {
                last_x_ = robot_x_;
                last_y_ = robot_y_;
                first_point_ = false;
            } else {
                double dist = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);
                if (dist > 1e-4) {
                    path_length_ += dist;
                    last_x_ = robot_x_;
                    last_y_ = robot_y_;
                }
            }
            session_positions_.emplace_back(robot_x_, robot_y_);
        }
    }

    void agents_callback(const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
        agent_positions_.clear();
        for (const auto &agent : msg->agent_states) {
            agent_positions_.emplace_back(agent.id,
                std::make_pair(agent.pose.position.x, agent.pose.position.y));
        }
    }

    void ellipses_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        ellipses_data_.assign(msg->data.begin(), msg->data.end());
    }

    void timer_callback() {
        if (!tracking_) return;

        double dt = (now() - last_time_).seconds();
        last_time_ = now();

        const double safety_radius = 1.2;

        for (const auto &[id, pos] : agent_positions_) {
            double dist = std::hypot(pos.first - robot_x_, pos.second - robot_y_);

            if (min_distances_per_agent_.find(id) == min_distances_per_agent_.end()) {
                min_distances_per_agent_[id] = dist;
            } else {
                min_distances_per_agent_[id] = std::min(min_distances_per_agent_[id], dist);
            }
            min_distance_ = std::min(min_distance_, dist);

            if (dist <= safety_radius) {
                violation_time_per_agent_[id] += dt;
            }
        }
    }

    void handle_nav_result(const GoalHandleNavigateToPose::WrappedResult &result) {
        std::string goal_status;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED: goal_status = "Succeeded"; break;
            case rclcpp_action::ResultCode::ABORTED: goal_status = "Failed"; break;
            case rclcpp_action::ResultCode::CANCELED: goal_status = "Canceled"; break;
            default: goal_status = "Unknown"; break;
        }

        RCLCPP_INFO(this->get_logger(), "Navigation finished with status: %s", goal_status.c_str());

        if (tracking_) {
            tracking_ = false;
            double total_time = (now() - start_time_).seconds();
            write_all_to_csv(path_length_, min_distance_, total_time, goal_status);
            write_positions_to_csv();
            RCLCPP_INFO(this->get_logger(), "=== END SESSION %d (%s) ===", session_id_, goal_status.c_str());
        }
    }

    void start_new_session() {
        session_id_++;
        tracking_ = true;
        first_point_ = true;
        path_length_ = 0.0;
        min_distance_ = std::numeric_limits<double>::infinity();
        start_time_ = now();
        last_time_ = start_time_;
        min_distances_per_agent_.clear();
        violation_time_per_agent_.clear();
        session_positions_.clear();

        std::ofstream(session_file_, std::ios::trunc) << session_id_;
        RCLCPP_INFO(this->get_logger(), "=== START NEW SESSION %d ===", session_id_);
    }

    void init_csv_headers() {
        std::ofstream pos(pos_csv_filename_, std::ios::app);
        if (pos.tellp() == 0) pos << "session_id,x,y\n";
        std::ofstream nav(nav_csv_filename_, std::ios::app);
        if (nav.tellp() == 0)
            nav << "session_id,PathLength(m),TotalTime(s),MinDistance(m),"
                   "PerAgent_MinDistance,PerAgent_ViolationTime,PerAgent_ViolationRate,GoalStatus\n";
    }

    void load_session_id() {
        std::ifstream in(session_file_);
        if (in.good()) in >> session_id_;
    }

    void write_all_to_csv(double path, double min_dist, double total_time, const std::string &status) {
        std::ofstream f(nav_csv_filename_, std::ios::app);
        std::stringstream s_min, s_vio, s_rate;
        for (auto &[id, dist] : min_distances_per_agent_) {
            double vio = violation_time_per_agent_[id];
            double rate = total_time > 0 ? vio / total_time * 100.0 : 0.0;
            s_min << "id" << id << ":" << dist << " ";
            s_vio << "id" << id << ":" << vio << " ";
            s_rate << "id" << id << ":" << rate << " ";
        }
        f << session_id_ << "," << path << "," << total_time << "," << min_dist << ",\""
          << s_min.str() << "\",\"" << s_vio.str() << "\",\"" << s_rate.str() << "\"," << status << "\n";
    }

    void write_positions_to_csv() {
        std::ofstream f(pos_csv_filename_, std::ios::app);
        for (auto &[x, y] : session_positions_) f << session_id_ << "," << x << "," << y << "\n";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationMetricsLogger>();

    rclcpp::on_shutdown([node]() {
        node->on_shutdown();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
