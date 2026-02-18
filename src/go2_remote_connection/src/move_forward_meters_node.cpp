#include <chrono>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class MoveForwardMeters : public rclcpp::Node {
public:
    MoveForwardMeters() : Node("move_forward_meters") {
        // ----------------- Parameters -----------------
        vx_ = this->declare_parameter<double>("vx", 0.25);
        hz_ = this->declare_parameter<double>("hz", 20.0);
        timeout_s_ = this->declare_parameter<double>("timeout_s", 12.0);
        max_m_ = this->declare_parameter<double>("max_m", 5.0);
        
        teleop_pub = this->create_publisher<geometry_msgs::msg::Twist>("/web_teleop", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom", 10, std::bind(&MoveForwardMeters::onOdom, this, std::placeholders::_1));
        goal_sub_  = this->create_subscription<std_msgs::msg::Float32>(
            "/move_forward_meters", 10, std::bind(&MoveForwardMeters::onGoal, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / hz_),
            std::bind(&MoveForwardMeters::onTick, this));
        
        RCLCPP_INFO(get_logger(), "MoveForwardMeters node ready");
    
    }

    ~MoveForwardMeters() override {
        publishStop();
    }

private:
    // ---------------- ROS Interfaces ----------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- Params and state ----
    double vx_{0.25}, hz_{20.0}, timeout_s_{12.0}, max_m_{5.0};
    double dir_{1.0};   // +1 forward, -1 backward

    // ---- Odom callback state ----
    bool have_odom_{false};
    double x_{0.0}, y_{0.0};

    // ---- Motion state ----
    bool active_{false};
    double goal_m_{0.0};
    double start_x_{0.0};
    double start_y_{0.0};
    rclcpp::Time start_time_;

    static double dist2D(double x1, double y1, double x2, double y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        have_odom_ = true;
    }

    void onGoal(const std_msgs::msg::Float32::SharedPtr msg) {

        double meters = static_cast<double>(msg->data);

        // Reject tiny commands (noise)
        const double eps = 1e-3;
        if (!std::isfinite(meters) || std::fabs(meters) < eps) {
          RCLCPP_WARN(get_logger(), "Received ~zero/invalid goal, ignoring");
          return;
        }
        
        // Clamp magnitude to max
        if (std::fabs(meters) > max_m_) {
          RCLCPP_WARN(get_logger(), "Goal %.2f m exceeds max %.2f m, clamping", meters, max_m_);
          meters = std::copysign(max_m_, meters);  // keep sign
        }
        
        if (!have_odom_) {
            RCLCPP_WARN(get_logger(), "Received goal but no odometry yet");
            return;
        }
        if (active_) {
            RCLCPP_WARN(get_logger(), "Received new goal while active, ignoring");
            return;
        }

        goal_m_ = meters;
        start_x_ = x_;
        start_y_ = y_;
        start_time_ = this->now();
        active_ = true;

        RCLCPP_INFO(get_logger(), "New goal: move forward %.2f meters", goal_m_);
    }

    void publishStop() {
        geometry_msgs::msg::Twist t{};
        t.linear.x = 0.0;
        t.linear.y = 0.0;
        t.angular.z = 0.0;
        teleop_pub->publish(t);
    }

    void publishCmd() {
        geometry_msgs::msg::Twist t{};
        t.linear.x = dir_ * vx_;   // dir_ flips sign        
        t.linear.y = 0.0;
        t.angular.z = 0.0;
        teleop_pub->publish(t);
    }

    void onTick() {
        if (!active_) return;

        const double travelled = dist2D(start_x_, start_y_, x_, y_);
        const double elapsed_s = (this->now() - start_time_).seconds();

        if (travelled >= goal_m_){
            RCLCPP_INFO(get_logger(), "Goal reached: moved %.2f m in %.2f s", travelled, elapsed_s);
            publishStop();
            active_ = false;
            return; 
        } else if (elapsed_s >= timeout_s_) {
            RCLCPP_WARN(get_logger(), "Timeout reached: moved %.2f m in %.2f s, stopping", travelled, elapsed_s);
            publishStop();
            active_ = false;
            return; 
        } else {
            publishCmd();
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveForwardMeters>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};