#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

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

    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/web_teleop_enabled", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        remote_enabled_.store(msg->data, std::memory_order_relaxed);
        // DO NOT send StopMove here (handheld friendly)
        was_active_ = false;
        zero_flush_left_ = 0;
        last_nonzero_ = false;
      });


    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridge::tick, this));
    last_rx_time_ = this->now();

    RCLCPP_INFO(get_logger(), "WebTeleopBridge started (EconomicGait)");
  }

private:
  // ---------------- Tunables ----------------
  static constexpr double VX_SCALE = 1.0;
  static constexpr double VY_SCALE = 1.0;
  static constexpr double VYAW_SCALE = 1.5;
  static constexpr double DEADBAND = 0.05;
  static constexpr double RX_TIMEOUT = 0.25;

  static constexpr double STANDUP_SETTLE_S = 1.6;
  static constexpr double GAIT_REASSERT_S  = 0.6;

  static constexpr int ZERO_FLUSH_TICKS = 10; // 10 * 50ms = 500ms

  // ---------------- State ----------------
  std::mutex mtx_;
  std::mutex sport_mtx_;

  double latest_vx_{0}, latest_vy_{0}, latest_vyaw_{0};
  rclcpp::Time last_rx_time_;

  bool was_active_{false};
  int zero_flush_left_{0};
  bool last_nonzero_{false};

  std::atomic<bool> remote_enabled_{true};
  bool posture_enabled_{true};
  std::atomic<int> pending_action_{0};

  bool gait_desired_{true};
  bool gait_sent_{false};
  rclcpp::Time stand_ready_time_{0,0,RCL_ROS_TIME};
  rclcpp::Time last_gait_cmd_time_{0,0,RCL_ROS_TIME};

  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Unitree ----------------
  SportClient sportClient_;

  // ---------------- Helpers ----------------
  void stopOnce() {
    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;
    sportClient_.StopMove(req);
    was_active_ = false;
    zero_flush_left_ = 0;
  }

  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "stand") pending_action_.store(1);
    else if (msg->data == "sit") pending_action_.store(2);
  }

  void doStand() {
    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    sportClient_.StopMove(req);
    sportClient_.StandUp(req);

    gait_desired_ = true;
    gait_sent_ = false;
    stand_ready_time_ = this->now() + rclcpp::Duration::from_seconds(STANDUP_SETTLE_S);

    posture_enabled_ = true;
    was_active_ = false;
    last_nonzero_ = false;
    zero_flush_left_ = 0;
  }

  void doSit() {
    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    sportClient_.StopMove(req);
    sportClient_.StandDown(req);

    gait_desired_ = false;
    posture_enabled_ = false;
    was_active_ = false;
    last_nonzero_ = false;
    zero_flush_left_ = 0;
  }

  void maybeAssertGait() {
    if (!gait_desired_) return;
    const auto now = this->now();
    if (now < stand_ready_time_) return;

    if (!gait_sent_) {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      unitree_api::msg::Request req;
      sportClient_.EconomicGait(req);
      gait_sent_ = true;
      last_gait_cmd_time_ = now;
    }
  }

  // ---------------- Main loop ----------------
  void tick() {
    if (!remote_enabled_.load(std::memory_order_relaxed)) return;

    const int act = pending_action_.exchange(0);
    if (act == 1) doStand();
    else if (act == 2) { doSit(); return; }

    maybeAssertGait();
    if (!posture_enabled_) return;

    const auto now = this->now();
    double vx, vy, vyaw;
    rclcpp::Time last_rx;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
      last_rx = last_rx_time_;
    }

    auto dz = [](double v){ return (std::abs(v) < DEADBAND) ? 0.0 : v; };
    vx = dz(vx); vy = dz(vy); vyaw = dz(vyaw);

    const bool timed_out = (now - last_rx).seconds() > RX_TIMEOUT;
    if (timed_out) vx = vy = vyaw = 0.0;

    const bool nonzero =
      (std::abs(vx) > 1e-6) || (std::abs(vy) > 1e-6) || (std::abs(vyaw) > 1e-6);

    // Detect joystick release: nonzero -> zero
    if (last_nonzero_ && !nonzero) {
      zero_flush_left_ = ZERO_FLUSH_TICKS;
    }
    last_nonzero_ = nonzero;

    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    if (nonzero) {
      was_active_ = true;
      sportClient_.Move(req,
        static_cast<float>(VX_SCALE * vx),
        static_cast<float>(VY_SCALE * -vy),
        static_cast<float>(VYAW_SCALE * -vyaw));
      return;
    }

    // If we just released, actively brake for a short window
    if (zero_flush_left_ > 0) {
      --zero_flush_left_;
      sportClient_.Move(req, 0.0f, 0.0f, 0.0f);
      return;
    }

    // After flush, send StopMove once then stay silent
    if (was_active_) {
      was_active_ = false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridge>());
  rclcpp::shutdown();
  return 0;
}
