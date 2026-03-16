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
#include "go2_remote_connection/common/ros2_sport_client.h"

using namespace std::chrono_literals;

class WebUnifiedBridge : public rclcpp::Node {
public:
  WebUnifiedBridge()
  : Node("web_unified_bridge"),
    sportClient_(this)
  {
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/web_teleop", 10,
      std::bind(&WebUnifiedBridge::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_action", 10,
      std::bind(&WebUnifiedBridge::actionCb, this, std::placeholders::_1));

    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/web_teleop_enabled", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        remote_enabled_.store(msg->data, std::memory_order_relaxed);

        // Reset state to avoid jerks when re-enabled
        was_active_ = false;
        zero_flush_left_ = 0;
        last_nonzero_ = false;
        last_rx_time_ = this->now();
      });

    timer_ = this->create_wall_timer(50ms, std::bind(&WebUnifiedBridge::tick, this));
    last_rx_time_ = this->now();

    // Default: MOVEMENT and StaticWalk like your advanced controller.
    {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      unitree_api::msg::Request req;
      sportClient_.StaticWalk(req);
    }

    RCLCPP_INFO(get_logger(), "WebUnifiedBridge started (movement/posing/actions + teleop feel)");
  }

private:
  // ---------------- Mode ----------------
  enum class Mode { MOVEMENT, POSING, ACTIONS };
  Mode mode_{Mode::MOVEMENT};

  // ---------------- Teleop tuning ----------------
  static constexpr double DEADBAND   = 0.05;
  static constexpr double RX_TIMEOUT = 0.25;

  static constexpr int ZERO_FLUSH_TICKS = 10;   // 10 * 50ms = 500ms

  static constexpr double VX_SCALE   = 1.0;
  static constexpr double VY_SCALE   = 1.0;
  static constexpr double VYAW_SCALE = 1.5;

  // Stand/gait reassert (from your WebTeleopBridge)
  static constexpr double STANDUP_SETTLE_S = 1.6;

  // Posing scaling (tune to taste)
  static constexpr float MAX_ROLL  = 0.6f;
  static constexpr float MAX_PITCH = 0.6f;
  static constexpr float MAX_YAW   = 0.8f;

  // ---------------- State ----------------
  std::mutex mtx_;
  std::mutex sport_mtx_;

  double latest_vx_{0}, latest_vy_{0}, latest_vyaw_{0};
  rclcpp::Time last_rx_time_;

  bool was_active_{false};
  int  zero_flush_left_{0};
  bool last_nonzero_{false};

  std::atomic<bool> remote_enabled_{true};

  // Posing state
  bool pose_flag_{false};

  // Advanced toggles/state
  bool freeBound_flag_{false};
  bool freeAvoid_flag_{false};
  bool crossStep_flag_{false};
  bool freeJump_flag_{false};
  bool walkUpright_flag_{false};
  bool classicWalk_flag_{false};
  bool handStand_flag_{false};
  bool is_sitting_{false};
  int  speed_level_index_{0}; // cycles -1,0,1

  // Gait reassert
  bool gait_desired_{true};
  bool gait_sent_{false};
  rclcpp::Time stand_ready_time_{0,0,RCL_ROS_TIME};

  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Unitree ----------------
  SportClient sportClient_;

  // ---------------- Helpers ----------------
  static inline double dz(double v) { return (std::abs(v) < DEADBAND) ? 0.0 : v; }

  bool specialLocomotionActive() const {
    return freeBound_flag_ || freeAvoid_flag_ || crossStep_flag_ ||
           freeJump_flag_ || walkUpright_flag_ || handStand_flag_ || classicWalk_flag_;
  }

  void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_vx_   = msg->linear.x;
    latest_vy_   = msg->linear.y;
    latest_vyaw_ = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void setMode(Mode m) {
    if (mode_ == m) return;

    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    // Exit posing cleanly
    if (mode_ == Mode::POSING && pose_flag_) {
      sportClient_.Pose(req, false);
      pose_flag_ = false;
    }

    mode_ = m;

    if (mode_ == Mode::MOVEMENT) {
      sportClient_.StaticWalk(req);
    } else if (mode_ == Mode::POSING) {
      sportClient_.StopMove(req);
      pose_flag_ = true;
      sportClient_.Pose(req, true);

      // Clear movement state so we never “brake spam” while posing
      was_active_ = false;
      zero_flush_left_ = 0;
      last_nonzero_ = false;
      last_rx_time_ = this->now();
    } else { // ACTIONS
      sportClient_.StaticWalk(req);
    }
  }

  void scheduleGaitReassert() {
    gait_desired_ = true;
    gait_sent_ = false;
    stand_ready_time_ = this->now() + rclcpp::Duration::from_seconds(STANDUP_SETTLE_S);
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
    }
  }

  // Accept BOTH your old and new action strings so all existing HTMLs keep working.
  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    // ----- mode switching (new) -----
    if (a == "mode_movement") { setMode(Mode::MOVEMENT); return; }
    if (a == "mode_posing")   { setMode(Mode::POSING);   return; }
    if (a == "mode_actions")  { setMode(Mode::ACTIONS);  return; }

    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    // ----- global helpers -----
    if (a == "stop_move")    { sportClient_.StopMove(req); return; }
    if (a == "static_walk")  { sportClient_.StaticWalk(req); return; }
    if (a == "trot_run")     { sportClient_.TrotRun(req); return; }
    if (a == "economic_gait"){ sportClient_.EconomicGait(req); gait_sent_ = true; return; }
    if (a == "switch_avoid") { sportClient_.SwitchAvoidMode(req); return; }

    // ----- old compatibility: stand/sit from WebTeleopBridge -----
    if (a == "stand") {
      sportClient_.StopMove(req);
      sportClient_.StandUp(req);
      is_sitting_ = false;
      scheduleGaitReassert();
      return;
    }
    if (a == "sit") {
      sportClient_.StopMove(req);
      sportClient_.StandDown(req);
      is_sitting_ = true;
      gait_desired_ = false;
      gait_sent_ = false;
      return;
    }

    // ----- actions mode (and compat) -----
    if (a == "sit_toggle") {
      if (!is_sitting_) { sportClient_.StandDown(req); is_sitting_ = true; }
      else              { sportClient_.StandUp(req);   is_sitting_ = false; scheduleGaitReassert(); }
      return;
    }

    if (a == "hello")        { sportClient_.Hello(req); return; }
    if (a == "stretch")      { sportClient_.Stretch(req); return; }
    if (a == "content")      { sportClient_.Content(req); return; }
    if (a == "heart")        { sportClient_.Heart(req); return; }
    if (a == "scrape")       { sportClient_.Scrape(req); return; }
    if (a == "front_pounce") { sportClient_.FrontPounce(req); return; }
    if (a == "front_jump")   { sportClient_.FrontJump(req); return; }
    if (a == "front_flip")   { sportClient_.FrontFlip(req); return; }
    if (a == "back_flip")    { sportClient_.BackFlip(req); return; }
    if (a == "left_flip")    { sportClient_.LeftFlip(req); return; }
    if (a == "recovery")     { sportClient_.RecoveryStand(req); return; }

    // ----- movement toggles -----
    if (a == "toggle_freebound")  { freeBound_flag_ = !freeBound_flag_; sportClient_.FreeBound(req, freeBound_flag_); return; }
    if (a == "toggle_freeavoid")  { freeAvoid_flag_ = !freeAvoid_flag_; sportClient_.FreeAvoid(req, freeAvoid_flag_); return; }
    if (a == "toggle_crossstep")  { crossStep_flag_ = !crossStep_flag_; sportClient_.CrossStep(req, crossStep_flag_); return; }
    if (a == "toggle_freejump")   { freeJump_flag_  = !freeJump_flag_;  sportClient_.FreeJump(req, freeJump_flag_);   return; }
    if (a == "toggle_walkupright"){ walkUpright_flag_= !walkUpright_flag_; sportClient_.WalkUpright(req, walkUpright_flag_); return; }
    if (a == "toggle_handstand")  { handStand_flag_ = !handStand_flag_; sportClient_.HandStand(req, handStand_flag_); return; }
    if (a == "toggle_classicwalk"){ classicWalk_flag_= !classicWalk_flag_; sportClient_.ClassicWalk(req, classicWalk_flag_); return; }

    if (a == "cycle_speed") {
      static const int SPEED_LEVELS[3] = {-1, 0, 1};
      speed_level_index_ = (speed_level_index_ + 1) % 3;
      sportClient_.SpeedLevel(req, SPEED_LEVELS[speed_level_index_]);
      return;
    }

    // ----- posing helpers -----
    if (a == "damp")         { sportClient_.Damp(req); return; }
    if (a == "balance_stand"){ sportClient_.BalanceStand(req); return; }

    // unknown: ignore
  }

  void tick() {
    if (!remote_enabled_.load(std::memory_order_relaxed)) return;

    maybeAssertGait();

    // read teleop values
    double vx, vy, vyaw;
    rclcpp::Time last_rx;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
      last_rx = last_rx_time_;
    }

    const auto now = this->now();

    vx = dz(vx);
    vy = dz(vy);
    vyaw = dz(vyaw);

    const bool timed_out = (now - last_rx).seconds() > RX_TIMEOUT;
    if (timed_out) vx = vy = vyaw = 0.0;

    // POSING: continuous Euler updates
    if (mode_ == Mode::POSING) {
      const float roll  = static_cast<float>(vy)   * MAX_ROLL;
      const float pitch = static_cast<float>(vx)   * MAX_PITCH;
      const float yaw   = static_cast<float>(vyaw) * -MAX_YAW;

      std::lock_guard<std::mutex> lk(sport_mtx_);
      unitree_api::msg::Request req;
      sportClient_.Euler(req, roll, pitch, yaw);

      // do NOT run movement stop/flush state machine in posing
      was_active_ = false;
      zero_flush_left_ = 0;
      last_nonzero_ = false;
      return;
    }

    // MOVEMENT + ACTIONS: teleop-feel Move/StopMove behavior
    const bool nonzero =
      (std::abs(vx) > 1e-6) || (std::abs(vy) > 1e-6) || (std::abs(vyaw) > 1e-6);

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

    if (zero_flush_left_ > 0) {
      --zero_flush_left_;
      sportClient_.Move(req, 0.0f, 0.0f, 0.0f);
      return;
    }

    if (was_active_) {
      was_active_ = false;

      // preserve special locomotion modes: avoid StopMove cancelling them
      if (!specialLocomotionActive()) {
        sportClient_.StopMove(req);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebUnifiedBridge>());
  rclcpp::shutdown();
  return 0;
}
