// #include <chrono>
// #include <string>
// #include <mutex>
// #include <atomic>
// #include <cmath>
// #include <algorithm> 

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "std_msgs/msg/string.hpp"

// #include "unitree_api/msg/request.hpp"
// #include "p2_remote_connection/common/ros2_sport_client.h"

// using namespace std::chrono_literals;

// class WebTeleopBridge : public rclcpp::Node {
// public:
//   WebTeleopBridge()
//   : Node("web_teleop_bridge"),
//     sportClient_(this)
//   {
//     teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//       "/web_teleop", 10,
//       std::bind(&WebTeleopBridge::teleopCb, this, std::placeholders::_1));

//     action_sub_ = this->create_subscription<std_msgs::msg::String>(
//       "/web_action", 10,
//       std::bind(&WebTeleopBridge::actionCb, this, std::placeholders::_1));

//     timer_ = this->create_wall_timer(
//       50ms, std::bind(&WebTeleopBridge::tick, this));

//     last_rx_time_ = this->now();

//     RCLCPP_INFO(get_logger(), "WebTeleopBridge started (SAFE MODE)");
//   }

// private:
//   // -------------------- Tunables --------------------
//   static constexpr double VX_SCALE   = 1.0;
//   static constexpr double VY_SCALE   = 1.0;
//   static constexpr double VYAW_SCALE = 1.5;
//   static constexpr double DEADBAND   = 0.05;
//   static constexpr double RX_TIMEOUT = 1.0;  

//   static constexpr double STANDUP_SETTLE_S = 1.8;   // wait after StandUp before forcing WalkUpright
//   static constexpr double UPRIGHT_REASSERT_S = 0.5; // periodically re-send WalkUpright when enabled
//   // -------------------- State --------------------
//   std::mutex mtx_;
//   double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
//   rclcpp::Time last_rx_time_;

//   bool was_active_{false};

//   // -------------------- ROS --------------------
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   // -------------------- Unitree --------------------
//   unitree_api::msg::Request req_;
//   SportClient sportClient_;

//   // -------------------- Callbacks --------------------
//   void teleopCb(const geometry_msgs::msg::Twist::SharedPtr msg) {

//     std::lock_guard<std::mutex> lk(mtx_);
//     latest_vx_   = msg->linear.x;
//     latest_vy_   = msg->linear.y;
//     latest_vyaw_= msg->angular.z;
//     last_rx_time_ = this->now();
//   }

//   void actionCb(const std_msgs::msg::String::SharedPtr msg) {
//     const auto &a = msg->data;

//     if (a == "stand") {
//       std::lock_guard<std::mutex> lk(sport_mtx_);

//       // Stop residual motion
//       sportClient_.StopMove(req_);

//       // Stand up
//       sportClient_.StandUp(req_);

//       // We *want* upright walk mode after the stand completes
//       upright_desired_ = true;
//       upright_sent_ = false;

//       // Delay before forcing WalkUpright(true) (stand takes time)
//       standup_ready_time_ = this->now() + rclcpp::Duration::from_seconds(STANDUP_SETTLE_S);

//       teleop_enabled_ = true;
//       was_active_ = false; // reset activity latch
//     } 
//     else if (a == "sit") {
//       std::lock_guard<std::mutex> lk(sport_mtx_);

//       // Stop motion and exit upright walk mode
//       sportClient_.StopMove(req_);
//       sportClient_.WalkUpright(req_, false);

//       upright_desired_ = false;
//       upright_sent_ = true;  // already sent false
//       last_upright_cmd_time_ = this->now();

//       // Sit down
//       sportClient_.StandDown(req_);

//       teleop_enabled_ = false;
//       was_active_ = false; // reset activity latch
//     }
//   }

//   // -------------------- Main loop --------------------
// //   void tick() {
// //     // // Tick rate:
// //     // static int count = 0;
// //     // static auto t0 = this->now();
// //     // count++;
// //     // auto dt = (this->now() - t0).seconds();
// //     // if (dt > 2.0) {
// //     //   RCLCPP_INFO(get_logger(), "tick rate ~ %.2f Hz", count / dt);
// //     //   t0 = this->now();
// //     //   count = 0;
// //     // }
    
// //     const auto now_t = this->now();
// //     double vx, vy, vyaw;

// //     rclcpp::Time last_rx;
// //     {
// //       std::lock_guard<std::mutex> lk(mtx_);
// //       vx = latest_vx_;
// //       vy = latest_vy_;
// //       vyaw = latest_vyaw_;
// //       last_rx = last_rx_time_;
// //     }

// //     auto dz = [](double v) {
// //       return (std::abs(v) < DEADBAND) ? 0.0 : v;
// //     };
// //     vx = dz(vx);
// //     vy = dz(vy);
// //     vyaw = dz(vyaw);

// //     const bool timed_out = (now_t - last_rx).seconds() > RX_TIMEOUT;
// //     // If timed out, force command to zero BUT still publish Move every tick
// //     if (timed_out) {
// //       vx = 0.0;
// //       vy = 0.0;
// //       vyaw = 0.0;
// //     }

// //     const bool stationary =
// //       std::abs(vx) < 1e-6 &&
// //       std::abs(vy) < 1e-6 &&
// //       std::abs(vyaw) < 1e-6;

// //     const bool active_now = !timed_out && !stationary;

// //     if (!active_now && was_active_) {
// //       sportClient_.StopMove(req_);
// //       was_active_ = false;
// //     } else if (active_now) {
// //       was_active_ = true;
// //     }

// //     sportClient_.Move(
// //       req_,
// //       static_cast<float>(VX_SCALE * vx),
// //       static_cast<float>(VY_SCALE * -vy),
// //       static_cast<float>(VYAW_SCALE * -vyaw)
// //     );
// //   }
// // };
//   void maybeAssertUpright() {
//     // Only relevant if we want upright true
//     if (!upright_desired_) return;

//     const auto now_t = this->now();

//     // Don't assert upright until stand motion should be complete
//     if (now_t < standup_ready_time_) return;

//     // Re-send WalkUpright(true) once after stand, and then periodically
//     const bool time_to_reassert =
//       (!upright_sent_) ||
//       ((now_t - last_upright_cmd_time_).seconds() > UPRIGHT_REASSERT_S);

//     if (!time_to_reassert) return;

//     std::lock_guard<std::mutex> lk(sport_mtx_);
//     sportClient_.WalkUpright(req_, true);
//     // zero move once to “enter” velocity-control cleanly
//     sportClient_.Move(req_, 0.0f, 0.0f, 0.0f);

//     upright_sent_ = true;
//     last_upright_cmd_time_ = now_t;
//   }

//   // -------------------- Main loop --------------------
//   void tick() {
//     // 1) Handle pending actions in ONE place (prevents racing sport calls)
//     const int act = pending_action_.exchange(0, std::memory_order_relaxed);
//     if (act == 1) {
//       RCLCPP_INFO(get_logger(), "Action: stand");
//       doStand();
//     } else if (act == 2) {
//       RCLCPP_INFO(get_logger(), "Action: sit");
//       doSit();
//       return; // we're sitting; don't bother with teleop in this tick
//     }

//     // 2) If we are (or just became) standing, ensure walk/upright mode is asserted
//     maybeAssertUpright();

//     // 3) If teleop is disabled (sitting), keep robot stopped and exit
//     if (!teleop_enabled_) {
//       std::lock_guard<std::mutex> lk(sport_mtx_);
//       sportClient_.StopMove(req_);
//       return;
//     }

//     // 4) Normal teleop stream
//     const auto now_t = this->now();
//     double vx, vy, vyaw;
//     rclcpp::Time last_rx;

//     {
//       std::lock_guard<std::mutex> lk(mtx_);
//       vx = latest_vx_;
//       vy = latest_vy_;
//       vyaw = latest_vyaw_;
//       last_rx = last_rx_time_;
//     }

//     auto dz = [](double v) { return (std::abs(v) < DEADBAND) ? 0.0 : v; };
//     vx = dz(vx);
//     vy = dz(vy);
//     vyaw = dz(vyaw);

//     const bool timed_out = (now_t - last_rx).seconds() > RX_TIMEOUT;
//     if (timed_out) {
//       vx = 0.0; vy = 0.0; vyaw = 0.0;
//     }

//     const bool stationary =
//       std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(vyaw) < 1e-6;

//     const bool active_now = !timed_out && !stationary;

//     // Only StopMove on transition from active->inactive
//     if (!active_now && was_active_) {
//       std::lock_guard<std::mutex> lk(sport_mtx_);
//       sportClient_.StopMove(req_);
//       was_active_ = false;
//     } else if (active_now) {
//       was_active_ = true;
//     }

//     // Always stream Move (Unitree likes continuous command stream)
//     {
//       std::lock_guard<std::mutex> lk(sport_mtx_);
//       sportClient_.Move(
//         req_,
//         static_cast<float>(VX_SCALE * vx),
//         static_cast<float>(VY_SCALE * -vy),
//         static_cast<float>(VYAW_SCALE * -vyaw)
//       );
//     }
//   }
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<WebTeleopBridge>());
//   rclcpp::shutdown();
//   return 0;
// }

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

    timer_ = this->create_wall_timer(50ms, std::bind(&WebTeleopBridge::tick, this));

    last_rx_time_ = this->now();

    RCLCPP_INFO(get_logger(), "WebTeleopBridge started");
  }

private:
  // -------------------- Tunables --------------------
  static constexpr double VX_SCALE   = 1.0;
  static constexpr double VY_SCALE   = 1.0;
  static constexpr double VYAW_SCALE = 1.5;
  static constexpr double DEADBAND   = 0.05;
  static constexpr double RX_TIMEOUT = 1.0;

  // These are the key additions:
  static constexpr double STANDUP_SETTLE_S = 1.8;   // wait after StandUp before forcing WalkUpright
  static constexpr double UPRIGHT_REASSERT_S = 0.5; // periodically re-send WalkUpright when enabled

  // -------------------- State --------------------
  std::mutex mtx_;        // protects latest velocities + last_rx
  std::mutex sport_mtx_;  // serializes ALL SportClient calls

  double latest_vx_{0.0}, latest_vy_{0.0}, latest_vyaw_{0.0};
  rclcpp::Time last_rx_time_;

  bool was_active_{false};

  // teleop / mode control
  std::atomic<int> pending_action_{0}; // 0=none, 1=stand, 2=sit
  bool teleop_enabled_{true};          // disabled while sitting
  bool upright_desired_{true};         // desired WalkUpright state
  bool upright_sent_{false};           // have we sent WalkUpright since last change?

  rclcpp::Time standup_ready_time_{0, 0, RCL_ROS_TIME}; // when it's safe to re-enter upright after stand
  rclcpp::Time last_upright_cmd_time_{0, 0, RCL_ROS_TIME};

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
    latest_vx_    = msg->linear.x;
    latest_vy_    = msg->linear.y;
    latest_vyaw_  = msg->angular.z;
    last_rx_time_ = this->now();
  }

  void actionCb(const std_msgs::msg::String::SharedPtr msg) {
    const auto &a = msg->data;
    if (a == "stand") pending_action_.store(1, std::memory_order_relaxed);
    else if (a == "sit") pending_action_.store(2, std::memory_order_relaxed);
  }

  // -------------------- Sport helpers --------------------
  void doStand() {
    std::lock_guard<std::mutex> lk(sport_mtx_);

    // Stop residual motion
    sportClient_.StopMove(req_);

    // Stand up
    sportClient_.StandUp(req_);

    // We *want* upright walk mode after the stand completes
    upright_desired_ = true;
    upright_sent_ = false;

    // Delay before forcing WalkUpright(true) (stand takes time)
    standup_ready_time_ = this->now() + rclcpp::Duration::from_seconds(STANDUP_SETTLE_S);

    teleop_enabled_ = true;
    was_active_ = false; // reset activity latch
  }

  void doSit() {
    std::lock_guard<std::mutex> lk(sport_mtx_);

    // Stop motion and exit upright walk mode
    sportClient_.StopMove(req_);
    sportClient_.WalkUpright(req_, false);

    upright_desired_ = false;
    upright_sent_ = true;  // already sent false
    last_upright_cmd_time_ = this->now();

    // Sit down
    sportClient_.StandDown(req_);

    teleop_enabled_ = false;
    was_active_ = false; // reset activity latch
  }

  void maybeAssertUpright() {
    // Only relevant if we want upright true
    if (!upright_desired_) return;

    const auto now_t = this->now();

    // Don't assert upright until stand motion should be complete
    if (now_t < standup_ready_time_) return;

    // Re-send WalkUpright(true) once after stand, and then periodically
    const bool time_to_reassert =
      (!upright_sent_) ||
      ((now_t - last_upright_cmd_time_).seconds() > UPRIGHT_REASSERT_S);

    if (!time_to_reassert) return;

    std::lock_guard<std::mutex> lk(sport_mtx_);
    sportClient_.WalkUpright(req_, true);
    // zero move once to “enter” velocity-control cleanly
    sportClient_.Move(req_, 0.0f, 0.0f, 0.0f);

    upright_sent_ = true;
    last_upright_cmd_time_ = now_t;
  }

  // -------------------- Main loop --------------------
  void tick() {
    // 1) Handle pending actions in ONE place (prevents racing sport calls)
    const int act = pending_action_.exchange(0, std::memory_order_relaxed);
    if (act == 1) {
      RCLCPP_INFO(get_logger(), "Action: stand");
      doStand();
    } else if (act == 2) {
      RCLCPP_INFO(get_logger(), "Action: sit");
      doSit();
      return; // we're sitting; don't bother with teleop in this tick
    }

    // 2) If we are (or just became) standing, ensure walk/upright mode is asserted
    maybeAssertUpright();

    // 3) If teleop is disabled (sitting), keep robot stopped and exit
    if (!teleop_enabled_) {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      sportClient_.StopMove(req_);
      return;
    }

    // 4) Normal teleop stream
    const auto now_t = this->now();
    double vx, vy, vyaw;
    rclcpp::Time last_rx;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = latest_vx_;
      vy = latest_vy_;
      vyaw = latest_vyaw_;
      last_rx = last_rx_time_;
    }

    auto dz = [](double v) { return (std::abs(v) < DEADBAND) ? 0.0 : v; };
    vx = dz(vx);
    vy = dz(vy);
    vyaw = dz(vyaw);

    const bool timed_out = (now_t - last_rx).seconds() > RX_TIMEOUT;
    if (timed_out) {
      vx = 0.0; vy = 0.0; vyaw = 0.0;
    }

    const bool stationary =
      std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(vyaw) < 1e-6;

    const bool active_now = !timed_out && !stationary;

    // Only StopMove on transition from active->inactive
    if (!active_now && was_active_) {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      sportClient_.StopMove(req_);
      was_active_ = false;
    } else if (active_now) {
      was_active_ = true;
    }

    // Always stream Move (Unitree likes continuous command stream)
    {
      std::lock_guard<std::mutex> lk(sport_mtx_);
      sportClient_.Move(
        req_,
        static_cast<float>(VX_SCALE * vx),
        static_cast<float>(VY_SCALE * -vy),
        static_cast<float>(VYAW_SCALE * -vyaw)
      );
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebTeleopBridge>());
  rclcpp::shutdown();
  return 0;
}
