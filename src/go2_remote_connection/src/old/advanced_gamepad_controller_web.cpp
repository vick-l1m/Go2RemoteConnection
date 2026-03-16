#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <cmath>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "unitree_api/msg/request.hpp"
#include "go2_remote_connection/common/ros2_sport_client.h"

using namespace std::chrono_literals;

class WebAdvancedBridge : public rclcpp::Node {
public:
  WebAdvancedBridge()
  : Node("web_advanced_bridge"),
    sportClient_(this)
  {
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/web_teleop", 10,
      std::bind(&WebAdvancedBridge::teleopCb, this, std::placeholders::_1));

    action_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/web_action", 10,
      std::bind(&WebAdvancedBridge::actionCb, this, std::placeholders::_1));

    enabled_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/web_teleop_enabled", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        remote_enabled_.store(msg->data, std::memory_order_relaxed);

        // Reset teleop state so we don’t “jerk” on resume
        was_active_ = false;
        zero_flush_left_ = 0;
        last_nonzero_ = false;
        last_rx_time_ = this->now();
      });

    timer_ = this->create_wall_timer(50ms, std::bind(&WebAdvancedBridge::tick, this));
    last_rx_time_ = this->now();

    // Start in MOVEMENT mode like your controller: StaticWalk
    {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      unitree_api::msg::Request req;
      sportClient_.StaticWalk(req);
    }

    RCLCPP_INFO(get_logger(), "WebAdvancedBridge started (modes: movement/posing/actions)");
  }

private:
  // ---------------- Mode ----------------
  enum class Mode { MOVEMENT, POSING, ACTIONS };
  Mode mode_{Mode::MOVEMENT};

  // ---------------- Teleop tuning (copied style from your web_teleop_bridge.cpp) ----------------
  static constexpr double DEADBAND   = 0.05;
  static constexpr double RX_TIMEOUT = 0.25;

  static constexpr int ZERO_FLUSH_TICKS = 10; // 500ms @50ms

  // Keep your “feel”
  static constexpr double VX_SCALE   = 1.0;
  static constexpr double VY_SCALE   = 1.0;
  static constexpr double VYAW_SCALE = 1.5;

  // Posing scaling (rough equivalents; tune to taste)
  static constexpr float MAX_ROLL  = 0.6f;   // rad-ish feel
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

  // Advanced flags / state
  bool pose_flag_{false};
  bool freeBound_flag_{false};
  bool freeAvoid_flag_{false};
  bool crossStep_flag_{false};
  bool freeJump_flag_{false};
  bool walkUpright_flag_{false};
  bool classicWalk_flag_{false};
  bool handStand_flag_{false};
  bool is_sitting_{false};
  int  speed_level_index_{0}; // -1,0,1 cycling

  bool specialLocomotionActive() const {
    return freeBound_flag_ || freeAvoid_flag_ || crossStep_flag_ ||
           freeJump_flag_ || walkUpright_flag_ || handStand_flag_ || classicWalk_flag_;
  }
  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Unitree ----------------
  SportClient sportClient_;

  // ---------------- Helpers ----------------
  static inline double dz(double v) { return (std::abs(v) < DEADBAND) ? 0.0 : v; }

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

    // Exiting posing -> Pose(false)
    if (mode_ == Mode::POSING && pose_flag_) {
      sportClient_.Pose(req, false);
      pose_flag_ = false;
    }

    mode_ = m;

    // Enter mode behavior matches your advanced controller
    if (mode_ == Mode::MOVEMENT) {
      sportClient_.StaticWalk(req);
    } else if (mode_ == Mode::POSING) {
        sportClient_.StopMove(req);        // stop translation immediately
        pose_flag_ = true;
        sportClient_.Pose(req, true);

        // IMPORTANT: clear movement state so we don't run the move state machine after pose
        was_active_ = false;
        zero_flush_left_ = 0;
        last_nonzero_ = false;

        // Optional: reset last_rx_time_ so timeout logic doesn't do weird stuff
        last_rx_time_ = this->now();
    } else { // ACTIONS
      sportClient_.StaticWalk(req);
    }
  }

  // Web actions come in here (same pipeline as your current stand/sit)
  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &a = msg->data;

    // Mode switching (buttons)
    if (a == "mode_movement") { setMode(Mode::MOVEMENT); return; }
    if (a == "mode_posing")   { setMode(Mode::POSING);   return; }
    if (a == "mode_actions")  { setMode(Mode::ACTIONS);  return; }

    // Commands depend on mode, but we also allow a few “global”
    std::lock_guard<std::mutex> lk(sport_mtx_);
    unitree_api::msg::Request req;

    // ---- Global-ish / common ----
    if (a == "stop_move") { sportClient_.StopMove(req); return; }
    if (a == "static_walk") { sportClient_.StaticWalk(req); return; }

    // ---- Movement mode commands (toggles + one-shots) ----
    if (a == "toggle_freebound") { freeBound_flag_ = !freeBound_flag_; sportClient_.FreeBound(req, freeBound_flag_); return; }
    if (a == "toggle_freeavoid") { freeAvoid_flag_ = !freeAvoid_flag_; sportClient_.FreeAvoid(req, freeAvoid_flag_); return; }
    if (a == "toggle_crossstep") { crossStep_flag_ = !crossStep_flag_; sportClient_.CrossStep(req, crossStep_flag_); return; }
    if (a == "toggle_freejump")  { freeJump_flag_  = !freeJump_flag_;  sportClient_.FreeJump(req, freeJump_flag_);   return; }
    if (a == "toggle_walkupright"){ walkUpright_flag_= !walkUpright_flag_; sportClient_.WalkUpright(req, walkUpright_flag_); return; }
    if (a == "toggle_handstand") { handStand_flag_ = !handStand_flag_; sportClient_.HandStand(req, handStand_flag_); return; }
    if (a == "toggle_classicwalk") { classicWalk_flag_ = !classicWalk_flag_; sportClient_.ClassicWalk(req, classicWalk_flag_); return;}

    if (a == "trot_run")      { sportClient_.TrotRun(req); return; }
    if (a == "economic_gait") { sportClient_.EconomicGait(req); return; }
    if (a == "switch_avoid")  { sportClient_.SwitchAvoidMode(req); return; }

    if (a == "cycle_speed") {
      static const int SPEED_LEVELS[3] = {-1, 0, 1};
      speed_level_index_ = (speed_level_index_ + 1) % 3;
      sportClient_.SpeedLevel(req, SPEED_LEVELS[speed_level_index_]);
      return;
    }

    // ---- Posing mode commands (one-shots) ----
    if (a == "damp")         { sportClient_.Damp(req); return; }
    if (a == "balance_stand"){ sportClient_.BalanceStand(req); return; }

    // ---- Actions mode commands (work like stand/sit: discrete “press -> action”) ----
    // Your advanced controller used A to toggle sit/stand; we expose explicit buttons.
    if (a == "sit_toggle") {
      if (!is_sitting_) { sportClient_.StandDown(req); is_sitting_ = true; }
      else              { sportClient_.StandUp(req);   is_sitting_ = false; }
      return;
    }
    if (a == "hello")       { sportClient_.Hello(req); return; }
    if (a == "stretch")     { sportClient_.Stretch(req); return; }
    if (a == "content")     { sportClient_.Content(req); return; }
    if (a == "heart")       { sportClient_.Heart(req); return; }
    if (a == "scrape")      { sportClient_.Scrape(req); return; }
    if (a == "front_pounce"){ sportClient_.FrontPounce(req); return; }
    if (a == "front_jump")  { sportClient_.FrontJump(req); return; }
    if (a == "front_flip")  { sportClient_.FrontFlip(req); return; }
    if (a == "back_flip")   { sportClient_.BackFlip(req); return; }
    if (a == "left_flip") { sportClient_.LeftFlip(req); return; }
    if (a == "recovery")    { sportClient_.RecoveryStand(req); return; }

    // Unknown action: ignore
  }

  void tick() {
    if (!remote_enabled_.load(std::memory_order_relaxed)) return;

    // Always read teleop
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

    // MODE behavior:
    if (mode_ == Mode::POSING) {
      // Same joystick fields you already send:
      // vx = linear.x, vy = linear.y, vyaw = angular.z
      //
      // We map them to Euler(roll,pitch,yaw) continuously:
      // roll from vy (strafe), pitch from vx (forward), yaw from vyaw
      const float roll  = static_cast<float>(vy)   * MAX_ROLL;
      const float pitch = static_cast<float>(vx)   * MAX_PITCH;
      const float yaw   = static_cast<float>(vyaw) * -MAX_YAW;

      std::lock_guard<std::mutex> lk(sport_mtx_);
      unitree_api::msg::Request req;
      sportClient_.Euler(req, roll, pitch, yaw);
      
      // IMPORTANT: ensure we don't "brake" via StopMove spam either
      was_active_ = false;
      zero_flush_left_ = 0;
      last_nonzero_ = false;

      return;
    }

    // MOVEMENT + ACTIONS: same Move/StopMove logic as your web_teleop_bridge.cpp
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

      // If a special locomotion mode is active, StopMove often cancels it.
      // So we do NOT StopMove — we simply stop sending commands.
      if (!specialLocomotionActive()) {
        sportClient_.StopMove(req);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebAdvancedBridge>());
  rclcpp::shutdown();
  return 0;
}
