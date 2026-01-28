#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm> 

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

    timer_ = this->create_wall_timer(
      50ms, std::bind(&WebTeleopBridge::tick, this));

    last_rx_time_ = this->now();

    RCLCPP_INFO(get_logger(), "WebTeleopBridge started (SAFE MODE)");
  }

private:
  // -------------------- Tunables --------------------
  static constexpr double VX_SCALE   = 1.0;
  static constexpr double VY_SCALE   = 1.0;
  static constexpr double VYAW_SCALE = 1.5;
  static constexpr double DEADBAND   = 0.05;
  static constexpr double RX_TIMEOUT = 1.0;  


  // -------------------- State --------------------
  std::mutex mtx_;
  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;

  bool was_active_{false};

  // -------------------- ROS --------------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // -------------------- Unitree --------------------
  unitree_api::msg::Request req_;
  SportClient sportClient_;

  // -------------------- Callbacks --------------------
  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {

    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_   = msg->linear.x;
    latest_vy_   = msg->linear.y;
    latest_vyaw_= msg->angular.z;
    last_rx_time_ = this->now();
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const auto &a = msg->data;

    if (a == "stand") {
      sportClient_.StandUp(req_);
    } 
    else if (a == "sit") {
      sportClient_.StandDown(req_);
    }
  }

  // -------------------- Main loop --------------------
  void tick() {
    // Tick rate:
    static int count = 0;
    static auto t0 = this->now();
    count++;
    auto dt = (this->now() - t0).seconds();
    if (dt > 2.0) {
      RCLCPP_INFO(get_logger(), "tick rate ~ %.2f Hz", count / dt);
      t0 = this->now();
      count = 0;
    }

    const auto now_t = this->now();
    double vx, vy, vyaw;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
      last_rx = last_rx_time_;
    }

    auto dz = [](double v) {
      return (std::abs(v) < DEADBAND) ? 0.0 : v;
    };
    vx = dz(vx);
    vy = dz(vy);
    vyaw = dz(vyaw);

    const bool timed_out = (now_t - last_rx_time_).seconds() > RX_TIMEOUT;
    // If timed out, force command to zero BUT still publish Move every tick
    if (timed_out) {
      vx = 0.0;
      vy = 0.0;
      vyaw = 0.0;
    }

    const bool stationary =
      std::abs(vx) < 1e-6 &&
      std::abs(vy) < 1e-6 &&
      std::abs(vyaw) < 1e-6;

    const bool active_now = !timed_out && !stationary;

    if (!active_now && was_active_) {
      sportClient_.StopMove(req_);
      was_active_ = false;
    } else if (active_now) {
      was_active_ = true;
    }

    sportClient_.Move(
      req_,
      static_cast<float>(VX_SCALE * vx),
      static_cast<float>(VY_SCALE * -vy),
      static_cast<float>(VYAW_SCALE * -vyaw)
    );
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridge>());
  rclcpp::shutdown();
  return 0;
}
