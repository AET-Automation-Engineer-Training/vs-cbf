#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <cmath>

#include "kalman.h"

using namespace std;
using namespace Eigen;

const size_t obs_kf_buffer_size = 100;

// Ellipse definition
struct Ellipse
{
    double semimajor; // Length of semi-major axis
    double semiminor; // Length of semi-minor axis
    double cx;        // x-coordinate of center
    double cy;        // y-coordinate of center
    double vx;        // x-velocity
    double vy;        // y-velocity
    double theta;     // Rotation angle
    int class_id; // Class ID
};

// Definition of obstacles
struct obs_param
{
    float x = 0.0;
    float y = 0.0;
    float vx = 0.0;
    float vy = 0.0;
    float a = 0.0;
    float b = 0.0;
    float theta = 0.0;
    float mea_cov;
    int class_id = 0; // Class ID, Default is 0 - Stranger
};

// Add a count of unused times, reset it if not used for a long time
class obs_kf
{
public:
    int N;
    int lastTimeUsed;
    float T;

    Kalman ka;
    std::vector<obs_param> param_list;
    std::vector<obs_param> pred_list;
    Eigen::VectorXd x;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::VectorXd z0;
    Eigen::VectorXd u_v;

    void obs_predict();
    obs_kf();
    void reset();
} _obs_kf[obs_kf_buffer_size];

obs_kf::obs_kf() : N(25)
{
    ka.m_StateSize = 9;
    // ka.m_MeaSize = 5; 
    ka.m_MeaSize = 7;
    // ka.m_MeaSize = 9; // For 3D bounding box with velocity
    ka.m_USize = 9;

    lastTimeUsed = 0;

    T = 0.06;

    x.resize(9);
    x.setZero();

    float P0_cov = 0.025;

    P0.resize(9, 9);
    P0.setIdentity();
    P0 *= P0_cov;

    float Q_cov = 0.00008;

    Q.resize(9, 9);
    Q.setIdentity();
    Q *= Q_cov;

    float R_cov = 0.000005; // Simulation 0.00005 Physical 0.000003

    R.resize(7, 7);
    R.setZero();

    R(0, 0) = R_cov;
    R(1, 1) = R_cov;
    R(2, 2) = 10 * R_cov;
    R(3, 3) = 10 * R_cov;
    R(4, 4) = 0.00000001 * R_cov;
    R(5, 5) = 5 * R_cov;  // hoặc giá trị phù hợp với tốc độ (vx)
    R(6, 6) = 5 * R_cov;  // hoặc giá trị phù hợp với tốc độ (vy)

    A.resize(9, 9);
    A.setIdentity();
    A(0, 2) = A(1, 3) = T;

    B.resize(9, 9);
    B.setZero();

    // H.resize(5, 9); 
    H.resize(7, 9); //  For 3D bounding box ưith velocity
    H.setIdentity();

    z0.resize(5);
    z0.setZero();

    u_v.resize(9);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::reset()
{
    ka.m_StateSize = 9;
    // ka.m_MeaSize = 5; 
    ka.m_MeaSize = 7;
    // ka.m_MeaSize = 9; // For 3D bounding box with velocity
    ka.m_USize = 9;

    lastTimeUsed = 0;

    param_list.clear();
    pred_list.clear();

    T = 0.06;

    x.resize(9);
    x.setZero();

    float P0_cov = 0.025;

    P0.resize(9, 9);
    P0.setIdentity();
    P0 *= P0_cov;

    float Q_cov = 0.00008;

    Q.resize(9, 9);
    Q.setIdentity();
    Q *= Q_cov;

    float R_cov = 0.000005; // Simulation 0.00005 Physical 0.000003

    R.resize(7, 7);
    R.setZero();

    R(0, 0) = R_cov;
    R(1, 1) = R_cov;
    R(2, 2) = 10 * R_cov;
    R(3, 3) = 10 * R_cov;
    R(4, 4) = 0.00000001 * R_cov;
    R(5, 5) = 5 * R_cov;  // hoặc giá trị phù hợp với tốc độ (vx)
    R(6, 6) = 5 * R_cov;  // hoặc giá trị phù hợp với tốc độ (vy)

    A.resize(9, 9);
    A.setIdentity();
    A(0, 2) = A(1, 3) = T;

    B.resize(9, 9);
    B.setZero();

    H.resize(5, 9);
    H.setIdentity();

    z0.resize(5);
    z0.setZero();

    u_v.resize(9);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::obs_predict()
{
    int num = param_list.size();
    if (num == 0)
        return;

    if (num == 1)
    {
        // ka.m_x << param_list[0].x, param_list[0].y, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0, 0, 0;
        ka.m_x << param_list[0].x, param_list[0].y, param_list[0].vx, param_list[0].vy, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0;
        ka.m_P = P0;
    }
    else
    {
        float _mea_cov_min = 0.005;
        float _mea_cov_max = 0.05;
        float _k_cov_min = 1;
        float _k_cov_max = 3000;
        float _mea_cov = min(max(param_list.back().mea_cov, _mea_cov_min), _mea_cov_max);
        float _k = log10(_mea_cov / _mea_cov_min) / log10(_mea_cov_max / _mea_cov_min);
        float _k_cov = pow(_k_cov_max, _k) * pow(_k_cov_min, 1 - _k);

        ka.m_R(0, 0) = _k_cov * R(0, 0);
        ka.m_R(1, 1) = _k_cov * R(1, 1);
        ka.m_R(2, 2) = _k_cov * R(0, 0); // vx
        ka.m_R(3, 3) = _k_cov * R(1, 1); // vy

        // VectorXd z(5);
        // z << param_list.back().x, param_list.back().y, param_list.back().a, param_list.back().b, param_list.back().theta;

        VectorXd z(7);
        z << param_list.back().x, param_list.back().y, param_list.back().vx, param_list.back().vy, param_list.back().a, param_list.back().b, param_list.back().theta;

        param_list.erase(param_list.begin());

        ka.Predict_State();
        ka.Predict_Cov();
        ka.Mea_Resd(z);
        ka.Cal_Gain();
        ka.Update_State();
        ka.Update_Cov();

        Kalman ka_tmp = ka;

        float scale_start = 0.0;
        float scale_end = 1.5;

        pred_list.clear();
        for (int i = 0; i < N; i++)
        {
            // float scale = scale_start + (scale_end - scale_start) * (static_cast<float>(i) / (N - 1));
            // std::cout << "scale = " << scale << " !!!!!" << std::endl;
            float scale = 4.0;

            ka_tmp.Predict_State();
            ka_tmp.Predict_Cov();
            obs_param pred;
            float x_conf = 0.95 * sqrt(ka_tmp.m_P(0, 0));
            float y_conf = 0.95 * sqrt(ka_tmp.m_P(1, 1));
            float vx_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2));
            float vy_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3));
            float a_conf = 0.95 * sqrt(ka_tmp.m_P(4, 4));
            float b_conf = 0.95 * sqrt(ka_tmp.m_P(5, 5));
            float theta_conf = 0.95 * sqrt(ka_tmp.m_P(6, 6));

            float pos_conf = max(x_conf, y_conf);
            float ab_conf = max(a_conf, b_conf);

            pred.x = ka_tmp.m_x(0);
            pred.y = ka_tmp.m_x(1);

            pred.vx = ka_tmp.m_x(2);
            pred.vy = ka_tmp.m_x(3);

            pred.a = (ka_tmp.m_x(4) + pos_conf + ab_conf) * scale * 0.7;
            pred.b = (ka_tmp.m_x(5) + pos_conf + ab_conf) * scale * 1.1;
            pred.theta = ka_tmp.m_x(6);
            pred.class_id = 0;

            pred_list.push_back(pred);
        }
    }
}

class ObsKfNode : public rclcpp::Node
{
public:
    ObsKfNode() : Node("obs_param_node")
    {
        unused_clear_time = 3;
        obs_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/for_obs_track", 1, std::bind(&ObsKfNode::obscb, this, std::placeholders::_1));
        
        obs_sub_3dbb_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/for_obs_track_3dbb", 1, std::bind(&ObsKfNode::obscb3dbb, this, std::placeholders::_1));

        obs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obs_predict_pub", 1);
        obs_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obs_predict_vis_pub", 1);
    }

private:
    void obscb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty())
            return;

        int num = msg->data.size() / 9;
        std_msgs::msg::Float32MultiArray obs_pub;
        std::vector<Ellipse> ellipses_array;

        for (int i = 0; i < obs_kf_buffer_size; ++i)
            _obs_kf[i].lastTimeUsed++;

        for (int i = 0; i < num; i++)
        {
            int base = i * 9;

            float x      = msg->data[base + 0];
            float y      = msg->data[base + 1];
            float vx     = msg->data[base + 2];
            float vy     = msg->data[base + 3];
            float a      = msg->data[base + 4];
            float b      = msg->data[base + 5];
            float theta  = msg->data[base + 6];
            int flag     = static_cast<int>(msg->data[base + 7]);
            float mea_cov = msg->data[base + 8];

            if (flag >= obs_kf_buffer_size || flag < 0)
            {
                RCLCPP_WARN(this->get_logger(), "label overflow %d (buffer size = %d)", flag, obs_kf_buffer_size);
                continue;
            }

            obs_param _obs_tmp;
            _obs_tmp.x = x;
            _obs_tmp.y = y;
            _obs_tmp.vx = vx;
            _obs_tmp.vy = vy;
            _obs_tmp.a = a;
            _obs_tmp.b = b;
            _obs_tmp.theta = theta;
            _obs_tmp.mea_cov = mea_cov;

            if (_obs_kf[flag].lastTimeUsed > unused_clear_time)
                _obs_kf[flag].reset();
            else
                _obs_kf[flag].lastTimeUsed = 0;
            _obs_kf[flag].param_list.push_back(_obs_tmp);

            curve_fitting(_obs_kf[flag], obs_pub, ellipses_array);
        }

        obs_pub_->publish(obs_pub);
        visEllipse(ellipses_array);
    }

    void obscb3dbb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[node] receive 3D bounding boxes for obstacle tracking");
        if (msg->data.empty()) return;

        const int fields_per_object = 8;  // x, y, z, l, w, theta, label, class_id
        int num_objects = msg->data.size() / fields_per_object;
        RCLCPP_INFO(this->get_logger(), "Processing %d obstacles", num_objects);

        std_msgs::msg::Float32MultiArray obs_pub;
        vector<Ellipse> ellipses_array;

        for (int i = 0; i < obs_kf_buffer_size; ++i)
            _obs_kf[i].lastTimeUsed++;

        for (int i = 0; i < num_objects; ++i)
        {
            // Extract fields
            float x = msg->data[fields_per_object * i + 0];
            float y = msg->data[fields_per_object * i + 1];
            float z = msg->data[fields_per_object * i + 2];
            float l = msg->data[fields_per_object * i + 3];
            float w = msg->data[fields_per_object * i + 4];
            float theta = msg->data[fields_per_object * i + 5];
            int label = static_cast<int>(msg->data[fields_per_object * i + 6]);
            int class_id = static_cast<int>(msg->data[fields_per_object * i + 7]);

            if (label >= obs_kf_buffer_size || label < 0) {
                RCLCPP_WARN(this->get_logger(), "Label overflow: %d", label);
                continue;
            }

            // Convert bbox to ellipse in 2D
            obs_param obs_tmp;
            obs_tmp.x = x;
            obs_tmp.y = y;
            obs_tmp.a = l / 2.0f;
            obs_tmp.b = w / 2.0f;
            obs_tmp.theta = theta;
            obs_tmp.class_id = class_id;
            obs_tmp.mea_cov = 0.1f;  // Optional: can be tuned based on sensor

            // Manage Kalman buffer
            if (_obs_kf[label].lastTimeUsed > unused_clear_time) {
                _obs_kf[label].reset();
            } else {
                _obs_kf[label].lastTimeUsed = 0;
            }

            _obs_kf[label].param_list.push_back(obs_tmp);

            // Generate predictions and visualization
            curve_fitting(_obs_kf[label], obs_pub, ellipses_array);
        }

        obs_pub_->publish(obs_pub);
        visEllipse(ellipses_array);
    }

    void obscb_with_velecity(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[node] receive 3D bounding boxes for obstacle tracking");
        if (msg->data.empty()) return;

        const int fields_per_object = 10;  // x, y, z, l, w, theta, label, class_id, vx, vy
        int num_objects = msg->data.size() / fields_per_object;
        RCLCPP_INFO(this->get_logger(), "Processing %d obstacles", num_objects);

        std_msgs::msg::Float32MultiArray obs_pub;
        vector<Ellipse> ellipses_array;

        for (int i = 0; i < obs_kf_buffer_size; ++i)
            _obs_kf[i].lastTimeUsed++;

        for (int i = 0; i < num_objects; ++i)
        {
            float x = msg->data[fields_per_object * i + 0];
            float y = msg->data[fields_per_object * i + 1];
            float z = msg->data[fields_per_object * i + 2];
            float l = msg->data[fields_per_object * i + 3];
            float w = msg->data[fields_per_object * i + 4];
            float theta = msg->data[fields_per_object * i + 5];
            int label = static_cast<int>(msg->data[fields_per_object * i + 6]);
            int class_id = static_cast<int>(msg->data[fields_per_object * i + 7]);
            float vx = msg->data[fields_per_object * i + 8];
            float vy = msg->data[fields_per_object * i + 9];

            if (label >= obs_kf_buffer_size || label < 0) {
                RCLCPP_WARN(this->get_logger(), "Label overflow: %d", label);
                continue;
            }

            obs_param obs_tmp;
            obs_tmp.x = x;
            obs_tmp.y = y;
            obs_tmp.a = l / 2.0f;
            obs_tmp.b = w / 2.0f;
            obs_tmp.vx = vx;
            obs_tmp.vy = vy;
            obs_tmp.theta = std::atan2(vy, vx);  // Use velocity for orientation
            obs_tmp.class_id = class_id;
            obs_tmp.mea_cov = 0.1f;

            if (_obs_kf[label].lastTimeUsed > unused_clear_time) {
                _obs_kf[label].reset();
            } else {
                _obs_kf[label].lastTimeUsed = 0;
            }

            _obs_kf[label].param_list.push_back(obs_tmp);
            curve_fitting(_obs_kf[label], obs_pub, ellipses_array);
        }

        obs_pub_->publish(obs_pub);
        visEllipse(ellipses_array);
    }

    void curve_fitting(obs_kf &obs, std_msgs::msg::Float32MultiArray &obs_pub, vector<Ellipse> &ellipses_array)
    {
        if (obs.param_list.empty())
            return;

        obs.obs_predict();

        for (int i = 0; i < obs.pred_list.size(); i++)
        {
            Ellipse ellipse;
            ellipse.cx = obs.pred_list[i].x;
            ellipse.cy = obs.pred_list[i].y;
            ellipse.vx = obs.pred_list[i].vx; // Use velocity for ellipse center
            ellipse.vy = obs.pred_list[i].vy; // Use velocity for ellipse center
            ellipse.semimajor = obs.pred_list[i].a;
            ellipse.semiminor = obs.pred_list[i].b;
            ellipse.theta = obs.pred_list[i].theta;
            ellipses_array.push_back(ellipse);

            obs_pub.data.push_back(obs.pred_list[i].x);
            obs_pub.data.push_back(obs.pred_list[i].y);
            obs_pub.data.push_back(obs.pred_list[i].vx); // Use velocity for ellipse center
            obs_pub.data.push_back(obs.pred_list[i].vy); // Use velocity for ellipse center
            obs_pub.data.push_back(obs.pred_list[i].a);
            obs_pub.data.push_back(obs.pred_list[i].b);
            obs_pub.data.push_back(obs.pred_list[i].theta);
            obs_pub.data.push_back(obs.pred_list[i].class_id);
        }
    }

    double offset_x_ = 0.0; 
    double offset_y_ = 0.0;
    // double offset_x_ = -7.0; 
    // double offset_y_ = -7.0;
    void visEllipse(const std::vector<Ellipse> &obs_ellipses)
    {
        visualization_msgs::msg::MarkerArray ellipse_vis;

        for (const auto &ellipse : obs_ellipses)
        {
            visualization_msgs::msg::Marker mk;
            mk.header.frame_id = "map";
            mk.header.stamp = this->now(); 
            mk.ns = "ellipse";
            mk.id = ellipse_vis.markers.size();
            mk.type = visualization_msgs::msg::Marker::CYLINDER;
            mk.action = visualization_msgs::msg::Marker::ADD;
            mk.lifetime = rclcpp::Duration::from_seconds(0.5); 

            mk.pose.position.x = ellipse.cx - offset_x_;
            mk.pose.position.y = ellipse.cy - offset_y_;
            mk.pose.position.z = -0.3;
            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = sin(ellipse.theta / 2);
            mk.pose.orientation.w = cos(ellipse.theta / 2);

            mk.scale.x = 1 * ellipse.semimajor;
            mk.scale.y = 1 * ellipse.semiminor;
            mk.scale.z = 0.7;

            mk.color.a = 0.4;
            mk.color.r = 0.0;
            mk.color.g = 1.0;
            mk.color.b = 0.0;

            ellipse_vis.markers.push_back(mk);
        }

        obs_vis_pub_->publish(ellipse_vis);
    }

    // Publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obs_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obs_sub_3dbb_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_vis_pub_;
    int unused_clear_time;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObsKfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}