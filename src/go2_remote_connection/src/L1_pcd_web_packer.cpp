#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <unordered_set>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

using std::placeholders::_1;

namespace
{
static inline bool finite3(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

// Pack float -> IEEE754 little-endian bytes (assumes platform is IEEE754, true on Jetson/x86)
static inline void pack_f32_le(float v, uint8_t *out)
{
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  std::memcpy(out, &v, 4);
}

// Hash for voxel key (ix,iy,iz)
struct VKey
{
  int32_t x, y, z;
  bool operator==(const VKey &o) const { return x==o.x && y==o.y && z==o.z; }
};

struct VKeyHash
{
  size_t operator()(const VKey& k) const noexcept
  {
    // a decent integer mix
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](uint32_t v){
      h ^= (uint64_t)v;
      h *= 1099511628211ull;
    };
    mix((uint32_t)k.x);
    mix((uint32_t)k.y);
    mix((uint32_t)k.z);
    return (size_t)h;
  }
};
} // namespace

class PcdWebPacker : public rclcpp::Node
{
public:
  PcdWebPacker()
  : Node("pcd_web_packer")
  {
    declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    declare_parameter<std::string>("target_frame", "odom");   // "" disables TF transform
    declare_parameter<int>("queue_size", 5);

    declare_parameter<double>("publish_hz", 5.0);

    declare_parameter<double>("voxel_size", 0.10);
    declare_parameter<int>("max_points", 20000);

    declare_parameter<double>("min_range", 0.25);
    declare_parameter<double>("max_range", 25.0);
    declare_parameter<double>("min_z", -1.0);
    declare_parameter<double>("max_z",  3.0);

    get_parameter("cloud_topic", cloud_topic_);
    get_parameter("target_frame", target_frame_);
    get_parameter("queue_size", queue_size_);
    get_parameter("publish_hz", publish_hz_);
    get_parameter("voxel_size", voxel_size_);
    get_parameter("max_points", max_points_);
    get_parameter("min_range", min_range_);
    get_parameter("max_range", max_range_);
    get_parameter("min_z", min_z_);
    get_parameter("max_z", max_z_);

    if (queue_size_ < 1) queue_size_ = 1;
    if (publish_hz_ < 0.5) publish_hz_ = 0.5;
    if (max_points_ < 100) max_points_ = 100;
    if (voxel_size_ <= 0.0) voxel_size_ = 0.0;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, queue_size_, std::bind(&PcdWebPacker::cloudCb, this, _1));

    // Meta is tiny; bytes can be large. Use reliable, small queue.
    meta_pub_  = create_publisher<std_msgs::msg::String>("/pcd/meta", rclcpp::QoS(5).reliable());
    bytes_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/pcd/xyz32", rclcpp::QoS(2).reliable());

    auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PcdWebPacker::tick, this));

    RCLCPP_INFO(get_logger(),
      "PcdWebPacker: %s -> /pcd/meta + /pcd/xyz32 (target_frame='%s', hz=%.1f, voxel=%.2f, max_points=%d)",
      cloud_topic_.c_str(), target_frame_.c_str(), publish_hz_, voxel_size_, max_points_);
  }

private:
  void cloudCb(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = msg;
    have_ = true;
  }

  void tick()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_ || !latest_) return;
      msg = latest_;
      have_ = false;
    }

    sensor_msgs::msg::PointCloud2 cloud_use;
    std::string out_frame = msg->header.frame_id;

    // Optional TF transform
    if (!target_frame_.empty())
    {
      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id,
          msg->header.stamp,
          tf2::durationFromSec(0.05));
        tf2::doTransform(*msg, cloud_use, tf);
        out_frame = target_frame_;
      } catch (const tf2::TransformException&) {
        cloud_use = *msg;
        out_frame = msg->header.frame_id;
      }
    }
    else
    {
      cloud_use = *msg;
      out_frame = msg->header.frame_id;
    }

    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;

    const double inv_vox = (voxel_size_ > 0.0) ? (1.0 / voxel_size_) : 0.0;

    std::unordered_set<VKey, VKeyHash> vox;
    if (voxel_size_ > 0.0) vox.reserve((size_t)max_points_ * 2);

    std::vector<uint8_t> packed;
    packed.reserve((size_t)max_points_ * 12);

    // Iterate xyz
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_use, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_use, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_use, "z");

    int n = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      float x = *it_x, y = *it_y, z = *it_z;
      if (!finite3(x,y,z)) continue;
      if (z < (float)min_z_ || z > (float)max_z_) continue;

      const double r2 = (double)x*(double)x + (double)y*(double)y;
      if (r2 < min_r2 || r2 > max_r2) continue;

      if (voxel_size_ > 0.0)
      {
        VKey k{
          (int32_t)std::floor((double)x * inv_vox),
          (int32_t)std::floor((double)y * inv_vox),
          (int32_t)std::floor((double)z * inv_vox),
        };
        if (vox.find(k) != vox.end()) continue;
        vox.insert(k);
      }

      // pack xyz little-endian float32
      uint8_t tmp[12];
      pack_f32_le(x, tmp + 0);
      pack_f32_le(y, tmp + 4);
      pack_f32_le(z, tmp + 8);
      packed.insert(packed.end(), tmp, tmp + 12);

      n++;
      if (n >= max_points_) break;
    }

    // Publish meta JSON
    seq_++;
    std_msgs::msg::String meta;
    meta.data =
      std::string("{\"t\":\"pcd\",\"seq\":") + std::to_string(seq_) +
      ",\"meta\":{\"frame_id\":\"" + out_frame +
      "\",\"fmt\":\"xyz32\",\"stride\":12},\"n\":" + std::to_string(n) + "}";

    std_msgs::msg::UInt8MultiArray bytes;
    bytes.data = std::move(packed);

    meta_pub_->publish(meta);
    bytes_pub_->publish(bytes);
  }

  // Params
  std::string cloud_topic_;
  std::string target_frame_;
  int queue_size_{5};
  double publish_hz_{5.0};
  double voxel_size_{0.10};
  int max_points_{20000};
  double min_range_{0.25};
  double max_range_{25.0};
  double min_z_{-1.0};
  double max_z_{3.0};

  // State
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_;
  bool have_{false};
  uint64_t seq_{0};

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr meta_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr bytes_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdWebPacker>());
  rclcpp::shutdown();
  return 0;
}
