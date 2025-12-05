#include <memory>
#include <vector>
#include <iostream>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <cmath>

class MujocoControlNode : public rclcpp::Node
{

public:
    MujocoControlNode() : Node("mujoco_control_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "sim_robot_state", 10,
            std::bind(&MujocoControlNode::state_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "sim_robot_cmd", 10);
    }

private:
    std::mutex mtx_control_;

    double gyro_data[3] = {0};
    double vel_data[3] = {0};

    double clamp(double val, double min_val, double max_val)
    {
        if (val > max_val)
            return max_val;
        if (val < min_val)
            return min_val;
        return val;
    }

    double quaternionToPitch(double w, double x, double y, double z)
    {
        double sinp = 2.0 * (w * y - z * x);
        sinp = clamp(sinp, -1.0, 1.0);
        return std::asin(sinp); // 返回弧度
    }
    
    double getYawFromQuaternion(double qx, double qy, double qz, double qw)
    {
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 14)
        {
            RCLCPP_WARN(this->get_logger(), "Received state size != 8");
            return;
        }

        double x = msg->data[0]; // x轴方向的位移
        double y = msg->data[1];
        double z = msg->data[2];
        double qw = msg->data[3]; // base_link 四元数
        double qx = msg->data[4];
        double qy = msg->data[5];
        double qz = msg->data[6];
        double CONTACT = msg->data[7]; // 接触 地面 标志位
        gyro_data[0] = msg->data[8];
        gyro_data[1] = msg->data[9]; // pitch 轴角速度
        gyro_data[2] = msg->data[10];
        vel_data[0] = msg->data[11]; // x 方向速度
        vel_data[1] = msg->data[12];
        vel_data[2] = msg->data[13];
        double pitch = quaternionToPitch(qw, qx, qy, qz);
        double yaw = getYawFromQuaternion(qx, qy, qz, qw);
        // ---------------- PD 控制器（针对 YAW 轴） ----------------
        static double prev_yaw_err = 0.0;
        const double Kp_yaw = 5.0; // 比例系数
        const double Kd_yaw = 0.5; // 微分系数
        double yaw_target = 0.0;   // 目标 YAW 为 0

        // 误差（当前 - 目标）
        double yaw_err = yaw_target - yaw;

        // 微分项（直接用 yaw 轴角速度代替）
        double yaw_rate = gyro_data[2];

        double pd_yaw_output = Kp_yaw * yaw_err - Kd_yaw * yaw_rate;
        prev_yaw_err = yaw_err;

        // ----------------------------------------------------------
        // [[ -4.47213595  -5.09635177 -19.39439063  -1.61452282]]
        double contorlL = 0.5 * (x * (3.65303708) + vel_data[0] * (3.99928814) + pitch * (18.01546699) + gyro_data[1] * (2.0045212));

        // RCLCPP_INFO(this->get_logger(), "vel_data[0] = %f", vel_data[0]);
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data.resize(2);
        std::lock_guard<std::mutex> lock(mtx_control_);
        {

            if (CONTACT == 0)
            {
                // 有碰撞 -> 停止
                cmd_msg.data = {0.0, 0.0};
            }
            else
            {
                // 没有碰撞 -> 前进
                cmd_msg.data = {contorlL+pd_yaw_output, contorlL-pd_yaw_output};
                //RCLCPP_INFO(this->get_logger(), "  %f ", contorlL);
            }
        }
        publisher_->publish(cmd_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MujocoControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}