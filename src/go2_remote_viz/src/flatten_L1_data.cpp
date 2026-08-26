#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>
#include <mutex>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/exceptions.h"

using std::placeholders::_1;

class FlattenL1Data : public rclcpp::Node
{
public:
  FlattenL1Data()
  : Node("flatten_l1_data")
  {
    // ---- Parameters ----
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<double>("max_width_m", 60.0);
    this->declare_parameter<double>("max_height_m", 60.0);
    this->declare_parameter<double>("voxel_res_multiplier", 2.0);

    this->declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    this->declare_parameter<std::string>("map2d_topic", "/l1/map2d");

    // If target_frame is empty, we do NOT TF-transform the cloud
    // (use this when subscribing to /utlidar/cloud_base, which is already in base frame).
    this->declare_parameter<std::string>("target_frame", "");
    this->declare_parameter<std::string>("base_frame", "base_link");

    this->declare_parameter<int>("queue_size", 5);
    this->declare_parameter<double>("update_radius_m", 10.0);
    this->declare_parameter<double>("tick_rate_hz", 5.0);
    this->declare_parameter<int>("max_occ", 100);
    this->declare_parameter<int>("full_map_publish_every_n_ticks", 10);
    this->declare_parameter<int>("decay_per_tick", 1);

    // Height filtering (relative to robot base Z, if TF available; else absolute Z in cloud frame)
    this->declare_parameter<double>("min_height_rel", 0.05);
    this->declare_parameter<double>("max_height_rel", 2.5);

    // ---- Get parameters ----
    this->get_parameter("resolution", resolution_);
    this->get_parameter("max_width_m", max_width_m_);
    this->get_parameter("max_height_m", max_height_m_);
    this->get_parameter("voxel_res_multiplier", voxel_res_multiplier_);
    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("map2d_topic", map2d_topic_);
    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("update_radius_m", update_radius_m_);
    this->get_parameter("tick_rate_hz", tick_rate_hz_);
    this->get_parameter("max_occ", max_occ_);
    this->get_parameter("full_map_publish_every_n_ticks", full_map_skip_);
    this->get_parameter("min_height_rel", min_height_rel_);
    this->get_parameter("max_height_rel", max_height_rel_);
    this->get_parameter("decay_per_tick", decay_per_tick_);


    if (max_occ_ < 1) max_occ_ = 1;

    // ---- Map geometry ----
    global_w_ = static_cast<size_t>(std::ceil(max_width_m_ / resolution_));
    global_h_ = static_cast<size_t>(std::ceil(max_height_m_ / resolution_));
    origin_x_ = -0.5 * (double)global_w_ * resolution_;
    origin_y_ = -0.5 * (double)global_h_ * resolution_;

    occupancy_counts_.assign(global_w_ * global_h_, -1);
    last_hit_tick_.assign(global_w_ * global_h_, 0);
    active_indices_.reserve(20000);

    RCLCPP_INFO(this->get_logger(),
      "FlattenL1Data: %s -> %s (%zux%zu @ %.3fm)",
      cloud_topic_.c_str(), map2d_topic_.c_str(),
      global_w_, global_h_, resolution_);

    // ---- TF (optional) ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Subscriber ----
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, queue_size_,
      std::bind(&FlattenL1Data::cloudCallback, this, _1));

    // ---- Publishers ----
    // Full map: transient_local so late subscribers instantly receive it.
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      map2d_topic_, rclcpp::QoS(1).transient_local().reliable());

    // Updates: volatile
    updates_topic_ = map2d_topic_ + "_updates";
    update_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
      updates_topic_, rclcpp::QoS(10).reliable());

    // Prepare map message
    map_msg_.header.frame_id = map_frame_id();
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = global_w_;
    map_msg_.info.height = global_h_;
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.resize(global_w_ * global_h_, -1);

    // Initial publish so your web side can show "something"
    map_msg_.header.stamp = this->now();
    publisher_->publish(map_msg_);

    auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&FlattenL1Data::update, this));
  }

private:
  std::string map_frame_id() const {
    // If you transform into a target frame, use it; otherwise use the incoming cloud frame.
    if (!target_frame_.empty()) return target_frame_;
    return "l1_map";  // stable frame name for the occupancy grid
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_msg_ = msg;
    have_cloud_ = true;
  }

  void update()
  {
    tick_count_++;

    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_in;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_cloud_ || !cloud_msg_) return;
      cloud_msg_in = cloud_msg_;
      have_cloud_ = false;
    }

    // Decide whether to transform the cloud
    sensor_msgs::msg::PointCloud2 cloud_use;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    bool have_pose = false;

    if (!target_frame_.empty()) {
      try {
        // Transform cloud into target_frame_
        auto tf_cloud = tf_buffer_->lookupTransform(
          target_frame_, cloud_msg_in->header.frame_id,
          cloud_msg_in->header.stamp,
          tf2::durationFromSec(0.1));

        tf2::doTransform(*cloud_msg_in, cloud_use, tf_cloud);

        // Also attempt base pose (for height thresholds around robot)
        auto tf_base = tf_buffer_->lookupTransform(
          target_frame_, base_frame_,
          tf2::TimePointZero);

        cx = tf_base.transform.translation.x;
        cy = tf_base.transform.translation.y;
        cz = tf_base.transform.translation.z;
        have_pose = true;
      } catch (const tf2::TransformException&) {
        // If TF fails, fall back to raw cloud frame (still produces a map)
        cloud_use = *cloud_msg_in;
        have_pose = false;
      }
    } else {
      // No TF mode: assume cloud is already in a suitable frame
      cloud_use = *cloud_msg_in;
      have_pose = false;
    }

    // Height thresholds
    const double min_z = (have_pose ? (cz + min_height_rel_) : min_height_rel_);
    const double max_z = (have_pose ? (cz + max_height_rel_) : max_height_rel_);

    // ROI
    const double r2_update = update_radius_m_ * update_radius_m_;
    active_indices_.clear();

    // Iterate points
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_use, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_use, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_use, "z");

    int coarse_mult = static_cast<int>(voxel_res_multiplier_);
    if (coarse_mult < 1) coarse_mult = 1;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      const float px = *it_x;
      const float py = *it_y;
      const float pz = *it_z;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

      if (pz < min_z || pz > max_z) continue;

      // distance gating (if we have robot pose, gate around it; else around origin)
      const double dx = (double)px - cx;
      const double dy = (double)py - cy;
      if (dx*dx + dy*dy > r2_update) continue;

      int ix = static_cast<int>(((double)px - origin_x_) / resolution_);
      int iy = static_cast<int>(((double)py - origin_y_) / resolution_);

      int cix = (ix / coarse_mult) * coarse_mult;
      int ciy = (iy / coarse_mult) * coarse_mult;

      if (cix < 0 || cix >= (int)global_w_ || ciy < 0 || ciy >= (int)global_h_) continue;
      size_t idx = (size_t)ciy * global_w_ + (size_t)cix;
      active_indices_.push_back(idx);
    }

    std::sort(active_indices_.begin(), active_indices_.end());
    active_indices_.erase(std::unique(active_indices_.begin(), active_indices_.end()),
                          active_indices_.end());

    // Paint occupied counts
    for (size_t idx : active_indices_) {
      int c = occupancy_counts_[idx];
      if (c < 0) c = 0;
      c += 20;
      if (c > max_occ_) c = max_occ_;
      occupancy_counts_[idx] = (int8_t)c;
      last_hit_tick_[idx] = tick_count_;
    }

    // Decay near ROI (simple)
    // Compute ROI bounds in grid coords around (cx,cy) if pose, else around (0,0).
    const double refx = cx, refy = cy;
    int ci = static_cast<int>((refx - origin_x_) / resolution_);
    int cj = static_cast<int>((refy - origin_y_) / resolution_);
    int ri = static_cast<int>(update_radius_m_ / resolution_);

    int roi_min_x = std::max(0, ci - ri - 2);
    int roi_max_x = std::min((int)global_w_ - 1, ci + ri + 2);
    int roi_min_y = std::max(0, cj - ri - 2);
    int roi_max_y = std::min((int)global_h_ - 1, cj + ri + 2);

    for (int y = roi_min_y; y <= roi_max_y; ++y) {
      for (int x = roi_min_x; x <= roi_max_x; ++x) {
        double wx = origin_x_ + (x + 0.5) * resolution_;
        double wy = origin_y_ + (y + 0.5) * resolution_;
        const double dx = wx - refx;
        const double dy = wy - refy;
        if (dx*dx + dy*dy > r2_update) continue;

        size_t idx = (size_t)y * global_w_ + (size_t)x;

        if (last_hit_tick_[idx] == tick_count_) continue;

        int c = occupancy_counts_[idx];
        if (c <= 0) {
          if (c < 0) occupancy_counts_[idx] = 0;
          continue;
        }
        c -= decay_per_tick_;
        if (c < 0) c = 0;
        occupancy_counts_[idx] = (int8_t)c;
      }
    }

    // Publish update patch
    map_msgs::msg::OccupancyGridUpdate upd;
    upd.header.stamp = this->now();
    upd.header.frame_id = map_frame_id();
    upd.x = roi_min_x;
    upd.y = roi_min_y;
    upd.width = (roi_max_x - roi_min_x) + 1;
    upd.height = (roi_max_y - roi_min_y) + 1;
    upd.data.resize((size_t)upd.width * (size_t)upd.height);

    for (uint32_t y = 0; y < upd.height; ++y) {
      for (uint32_t x = 0; x < upd.width; ++x) {
        size_t g = (size_t)(y + roi_min_y) * global_w_ + (size_t)(x + roi_min_x);
        size_t l = (size_t)y * upd.width + (size_t)x;
        int8_t val = occupancy_counts_[g];
        if (val < 0) upd.data[l] = -1;
        else if (val == 0) upd.data[l] = 0;
        else {
          int v = (int)val * 100 / max_occ_;
          if (v > 100) v = 100;
          upd.data[l] = (int8_t)v;
        }
      }
    }
    update_pub_->publish(upd);

    // Publish full map occasionally (latched)
    if (tick_count_ % (uint32_t)full_map_skip_ == 0) {
      map_msg_.header.stamp = this->now();
      map_msg_.header.frame_id = map_frame_id();
      const size_t n = occupancy_counts_.size();
      for (size_t i = 0; i < n; ++i) {
        int8_t val = occupancy_counts_[i];
        if (val < 0) map_msg_.data[i] = -1;
        else if (val == 0) map_msg_.data[i] = 0;
        else {
          int v = (int)val * 100 / max_occ_;
          if (v > 100) v = 100;
          map_msg_.data[i] = (int8_t)v;
        }
      }
      publisher_->publish(map_msg_);
    }
  }

  // Params
  double resolution_{0.1};
  double max_width_m_{60.0};
  double max_height_m_{60.0};
  double voxel_res_multiplier_{2.0};
  double min_height_rel_{0.05};
  double max_height_rel_{2.5};

  std::string cloud_topic_{"/utlidar/cloud_deskewed"};
  std::string map2d_topic_{"/l1/map2d"};
  std::string updates_topic_;
  std::string target_frame_{""};
  std::string base_frame_{"base_link"};

  int queue_size_{5};
  double update_radius_m_{10.0};
  double tick_rate_hz_{5.0};
  int max_occ_{100};
  int full_map_skip_{10};
  int decay_per_tick_{1};

  size_t global_w_{0};
  size_t global_h_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};

  std::vector<int8_t> occupancy_counts_;
  std::vector<uint32_t> last_hit_tick_;
  std::vector<size_t> active_indices_;
  uint32_t tick_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex data_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_;
  bool have_cloud_{false};

  nav_msgs::msg::OccupancyGrid map_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlattenL1Data>());
  rclcpp::shutdown();
  return 0;
}
