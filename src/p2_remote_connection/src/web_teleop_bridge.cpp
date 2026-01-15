#include <chrono>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "unitree_api/msg/request.hpp"
#include "p2_remote_connection/common/ros2_sport_client.h"

using namespace std::chrono_literals;

class WebTeleopBridge : public rclcpp::Node {
public:
  WebTeleopBridge()
  : Node("web_teleop_bridge"),
    sportClient_(this)
  {
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/web_teleop", 10,
      std::bind(&WebTeleopBridge::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_action", 10,
      std::bind(&WebTeleopBridge::actionCb, this, std::placeholders::_1));

    // 50 ms ~= 20 Hz command stream (tune as needed)
    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridge::tick, this));

    last_rx_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "WebTeleopBridge started.");
  }

private:
  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    // You can add more mappings here (Hello, Stretch, etc.)
    if (a == "stand") {
      sportClient_.StandUp(req_);
    } else if (a == "sit") {
      sportClient_.StandDown(req_);
    } else if (a == "stop") {
      sportClient_.StopMove(req_);
    // } else if (a == "standdown") {
    //   sportClient_.StandDown(req_);
    // } else if (a == "recover") {
    //   sportClient_.RecoveryStand(req_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown action: '%s'", a.c_str());
    }
  }

  void tick() {
    // Watchdog: if no teleop received recently, stop
    const double timeout_s = 0.25; // 250 ms watchdog
    double age = (this->now() - last_rx_time_).seconds();

    double vx, vy, vyaw;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
    }

    if (age > timeout_s) {
      // No fresh command => STOP
      sportClient_.StopMove(req_);
      return;
    }

    // If moving, send Move, else StopMove (same logic as your gamepad)
    const bool stationary = (std::abs(vx) < 1e-3 && std::abs(vy) < 1e-3 && std::abs(vyaw) < 1e-3);
    if (stationary) {
      sportClient_.StopMove(req_);
    } else {
      sportClient_.Move(req_, (float)vx, (float)vy, (float)vyaw);
    }
  }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Unitree
  unitree_api::msg::Request req_;
  SportClient sportClient_;

  // State
  std::mutex mtx_;
  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridge>());
  rclcpp::shutdown();
  return 0;
}
