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

using namespace std::chrono_literals;

class WebTeleopBridgeSim : public rclcpp::Node {
public:
  WebTeleopBridgeSim()
  : Node("web_teleop_bridge_sim")
  {
    // Parameters (SIM-UNIQUE)
    this->declare_parameter<int>("robot_index", 0);
    this->declare_parameter<double>("timeout_s", 0.25);

    robot_index_ = this->get_parameter("robot_index").as_int();
    timeout_s_   = this->get_parameter("timeout_s").as_double();

    // Topics (SIM-UNIQUE - keep)
    web_teleop_topic_  = "/web_teleop";
    web_action_topic_  = "/web_action";
    enabled_topic_     = "/web_teleop_enabled";
    sim_cmd_vel_topic_ = "/robot" + std::to_string(robot_index_) + "/cmd_vel";

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      web_teleop_topic_, 10,
      std::bind(&WebTeleopBridgeSim::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      web_action_topic_, 10,
      std::bind(&WebTeleopBridgeSim::actionCb, this, std::placeholders::_1));

    // NEW (parity): remote enable/disable control
    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      enabled_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        remote_enabled_.store(msg->data, std::memory_order_relaxed);

        // Parity with physical: reset motion state on enable flips
        was_active_ = false;
        zero_flush_left_ = 0;
        last_nonzero_ = false;

        // In sim, also publish zero once to be safe
        publishZero();
      });

    // Keep your stairs detection subscriber
    stairs_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stairs_detected", 10,
      std::bind(&WebTeleopBridgeSim::stairsCb, this, std::placeholders::_1));

    // Publish to sim cmd_vel (SIM-UNIQUE)
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(sim_cmd_vel_topic_, 10);

    // 50 ms ~= 20 Hz loop (same cadence as physical)
    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridgeSim::tick, this));

    last_rx_time_ = this->now();
    last_stairs_true_ = this->now() - rclcpp::Duration::from_seconds(1000.0);

    RCLCPP_INFO(this->get_logger(),
      "WebTeleopBridgeSim started.\n"
      "  Sub: %s, %s, %s, /stairs_detected\n"
      "  Pub: %s\n"
      "  robot_index=%d timeout_s=%.2f",
      web_teleop_topic_.c_str(), web_action_topic_.c_str(), enabled_topic_.c_str(),
      sim_cmd_vel_topic_.c_str(), robot_index_, timeout_s_);
  }

private:
  // ---------------- Tunables (parity + keep SIM scaling) ----------------
  static constexpr double VX_SCALE   = 2.0;  // KEEP your SIM values
  static constexpr double VY_SCALE   = 2.0;
  static constexpr double VYAW_SCALE = 1.5;

  static constexpr double DEADBAND  = 0.05;  // parity
  static constexpr double RX_TIMEOUT = 0.25; // parity; we also keep param timeout_s_

  static constexpr int ZERO_FLUSH_TICKS = 10; // parity: 10 * 50ms = 500ms

  // --- Stairs mode (keep) ---
  static constexpr double STAIRS_FWD_VX  = 0.15;
  static constexpr double STAIRS_FWD_VY  = 0.00;
  static constexpr double STAIRS_FWD_YAW = 0.00;

  static constexpr double STAIRS_ENTER_HOLD_S = 0.30;
  static constexpr double STAIRS_EXIT_CLEAR_S = 1.50;

  // ---------------- State (parity) ----------------
  std::mutex mtx_;

  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;

  bool was_active_{false};
  int zero_flush_left_{0};
  bool last_nonzero_{false};

  std::atomic<bool> remote_enabled_{true};
  bool posture_enabled_{true};
  std::atomic<int> pending_action_{0}; // 1=stand, 2=sit

  // SIM stairs state
  bool stairs_mode_{false};
  rclcpp::Time last_stairs_true_;

  // Params (SIM-UNIQUE)
  int robot_index_{0};
  double timeout_s_{0.25};

  // Topics (SIM-UNIQUE)
  std::string web_teleop_topic_;
  std::string web_action_topic_;
  std::string enabled_topic_;
  std::string sim_cmd_vel_topic_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stairs_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Helpers ----------------
  static double deadzone(double v) {
    return (std::abs(v) < DEADBAND) ? 0.0 : v;
  }

  void publishZero() {
    geometry_msgs::msg::Twist t; // zero init
    cmd_pub_->publish(t);
  }

  void publishCmd(double vx, double vy, double vyaw) {
    geometry_msgs::msg::Twist t;
    t.linear.x  = vx;
    t.linear.y  = vy;
    t.angular.z = vyaw;
    cmd_pub_->publish(t);
  }

  // ---------------- Callbacks ----------------
  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_ = msg->linear.x;
    latest_vy_ = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void stairsCb(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      last_stairs_true_ = this->now();
    }
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "stand") pending_action_.store(1);
    else if (msg->data == "sit") pending_action_.store(2);
    else if (msg->data == "stop") pending_action_.store(3);
    else {
      // keep silent / optional log
    }
  }

  // ---------------- Parity "actions" (SIM-safe) ----------------
  void doStandSim() {
    // In sim, you may not have a posture controller.
    // We keep parity by enabling posture + resetting motion state.
    posture_enabled_ = true;

    was_active_ = false;
    last_nonzero_ = false;
    zero_flush_left_ = 0;

    // Optional: publish a clean zero at posture changes
    publishZero();
    RCLCPP_INFO(this->get_logger(), "[SIM] stand -> posture_enabled=true (no-op posture)");
  }

  void doSitSim() {
    posture_enabled_ = false;

    was_active_ = false;
    last_nonzero_ = false;
    zero_flush_left_ = 0;

    publishZero();
    RCLCPP_INFO(this->get_logger(), "[SIM] sit -> posture_enabled=false (teleop suppressed)");
  }

  void doStopSim() {
    // Parity-ish: stop cancels stairs mode and zeros cmd_vel
    if (stairs_mode_) {
      stairs_mode_ = false;
      RCLCPP_INFO(this->get_logger(), "[SIM] stop -> exit STAIRS mode");
    }
    publishZero();
  }

  // ---------------- Main loop ----------------
  void tick() {
    // Parity: if disabled, do nothing (or optionally keep publishing zero)
    if (!remote_enabled_.load(std::memory_order_relaxed)) {
      // publishing zero here can be spammy; leave it silent like the physical code
      return;
    }

    // Handle pending action (parity)
    const int act = pending_action_.exchange(0);
    if (act == 1) doStandSim();
    else if (act == 2) { doSitSim(); return; }
    else if (act == 3) { doStopSim(); return; }

    if (!posture_enabled_) return;

    const auto now = this->now();

    // --- KEEP: stairs mode enter/exit logic ---
    const double stairs_age = (now - last_stairs_true_).seconds();
    const bool stairs_seen_recently = (stairs_age < STAIRS_ENTER_HOLD_S);
    const bool stairs_clear_long_enough = (stairs_age > STAIRS_EXIT_CLEAR_S);

    if (!stairs_mode_ && stairs_seen_recently) {
      stairs_mode_ = true;
      RCLCPP_WARN(this->get_logger(), "[SIM] Entering STAIRS mode (override cmd_vel)");
    }
    if (stairs_mode_ && stairs_clear_long_enough) {
      stairs_mode_ = false;
      RCLCPP_WARN(this->get_logger(), "[SIM] Exiting STAIRS mode");
    }

    if (stairs_mode_) {
      publishCmd(STAIRS_FWD_VX, STAIRS_FWD_VY, STAIRS_FWD_YAW);
      return;
    }

    // --- Teleop watchdog + parity deadband/flush behavior ---
    double vx, vy, vyaw;
    rclcpp::Time last_rx;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
      last_rx = last_rx_time_;
    }

    // Apply parity deadband
    vx   = deadzone(vx);
    vy   = deadzone(vy);
    vyaw = deadzone(vyaw);

    // Watchdog: choose max(parity constant, param) so you can still tune via param
    const double age = (now - last_rx).seconds();
    const double effective_timeout = std::max(timeout_s_, RX_TIMEOUT);

    const bool timed_out = (age > effective_timeout);
    if (timed_out) {
      vx = vy = vyaw = 0.0;
    }

    const bool nonzero =
      (std::abs(vx) > 1e-6) || (std::abs(vy) > 1e-6) || (std::abs(vyaw) > 1e-6);

    // Detect joystick release: nonzero -> zero
    if (last_nonzero_ && !nonzero) {
      zero_flush_left_ = ZERO_FLUSH_TICKS;
    }
    last_nonzero_ = nonzero;

    if (nonzero) {
      was_active_ = true;

      // KEEP SIM axis conventions + scaling
      const double cmd_vx   = VX_SCALE   * vx;
      const double cmd_vy   = VY_SCALE   * -vy;
      const double cmd_vyaw = VYAW_SCALE * -vyaw;

      publishCmd(cmd_vx, cmd_vy, cmd_vyaw);
      return;
    }

    // Braking window after release
    if (zero_flush_left_ > 0) {
      --zero_flush_left_;
      publishZero();
      return;
    }

    // After flush, stay silent like physical (but we can optionally publish zero once)
    if (was_active_) {
      was_active_ = false;
      // Optional single zero already handled by flush; no extra publish needed
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridgeSim>());
  rclcpp::shutdown();
  return 0;
}
