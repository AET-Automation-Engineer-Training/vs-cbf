#include "rclcpp/rclcpp.hpp"
#include "pedsim_msgs/msg/line_obstacles.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <cmath>

class LineObstaclesToPointCloud : public rclcpp::Node
{
public:
    LineObstaclesToPointCloud()
        : Node("line_obstacles_to_point_cloud")
    {
        // Subscription to the LineObstacles topic
        line_obstacles_sub_ = this->create_subscription<pedsim_msgs::msg::LineObstacles>(
            "/pedsim_simulator/simulated_walls", 10,
            std::bind(&LineObstaclesToPointCloud::line_obstacles_callback, this, std::placeholders::_1)
        );

        // Publisher for PointCloud2 message
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/simulated_walls_pointcloud", 10);
    }

private:
    void line_obstacles_callback(const pedsim_msgs::msg::LineObstacles::SharedPtr msg)
    {
        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        point_cloud_msg.header.stamp = this->get_clock()->now();
        point_cloud_msg.header.frame_id = "map";  // Set the frame_id

        // Prepare the fields for PointCloud2
        point_cloud_msg.fields.resize(3);
        point_cloud_msg.fields[0].name = "x";
        point_cloud_msg.fields[0].offset = 0;
        point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[0].count = 1;

        point_cloud_msg.fields[1].name = "y";
        point_cloud_msg.fields[1].offset = 4;
        point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[1].count = 1;

        point_cloud_msg.fields[2].name = "z";
        point_cloud_msg.fields[2].offset = 8;
        point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        point_cloud_msg.fields[2].count = 1;

        point_cloud_msg.is_bigendian = false;
        point_cloud_msg.point_step = 12;  // 3 fields (x, y, z) of 4 bytes each
        point_cloud_msg.is_dense = true;

        // Initialize the point cloud data container
        std::vector<uint8_t> point_cloud_data;

        // Iterate through each obstacle line and add intermediate points
        for (const auto &line : msg->obstacles)
        {
            // Create a set of points along the line from start to end
            add_line_points(line.start.x, line.start.y, line.end.x, line.end.y, point_cloud_data);
        }

        // Set the width and height of the point cloud
        point_cloud_msg.width = point_cloud_data.size() / 12;  // Number of points (each point is 12 bytes)
        point_cloud_msg.height = 1;  // Flat point cloud

        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

        // Set the point cloud data
        point_cloud_msg.data = point_cloud_data;

        // Publish the PointCloud2 message
        point_cloud_pub_->publish(point_cloud_msg);
    }

    // Add points along the line segment from (x1, y1) to (x2, y2)
    void add_line_points(float x1, float y1, float x2, float y2, std::vector<uint8_t> &point_cloud_data)
    {
        const int num_points = 50;  // Number of points to divide the line into
        float step_x = (x2 - x1) / num_points;
        float step_y = (y2 - y1) / num_points;

        // Add points along the line
        for (int i = 0; i <= num_points; ++i)
        {
            float x = x1 + step_x * i;
            float y = y1 + step_y * i;
            encode_point(x, y, 0.0, point_cloud_data);
        }
    }

    // Encode a point into the point cloud data vector
    void encode_point(float x, float y, float z, std::vector<uint8_t> &point_cloud_data)
    {
        uint8_t buffer[12];
        memcpy(buffer, &x, sizeof(float));  // Copy x-coordinate
        memcpy(buffer + 4, &y, sizeof(float));  // Copy y-coordinate
        memcpy(buffer + 8, &z, sizeof(float));  // Copy z-coordinate

        point_cloud_data.insert(point_cloud_data.end(), buffer, buffer + 12);  // Append the point to the data
    }

    rclcpp::Subscription<pedsim_msgs::msg::LineObstacles>::SharedPtr line_obstacles_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineObstaclesToPointCloud>());
    rclcpp::shutdown();
    return 0;
}
