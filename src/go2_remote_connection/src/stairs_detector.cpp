#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"

class StairsDetector : public rclcpp::Node {
public:
  StairsDetector() : Node("stairs_detector") {
    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");

    // ROI in front of robot (meters)
    x_min_ = this->declare_parameter<double>("x_min", 0.25);
    x_max_ = this->declare_parameter<double>("x_max", 1.20);
    y_min_ = this->declare_parameter<double>("y_min", -0.25);
    y_max_ = this->declare_parameter<double>("y_max",  0.25);
    z_min_ = this->declare_parameter<double>("z_min", -0.30);
    z_max_ = this->declare_parameter<double>("z_max",  0.60);

    bin_size_ = this->declare_parameter<double>("bin_size", 0.05);          // 5cm bins
    min_points_per_bin_ = this->declare_parameter<int>("min_points_per_bin", 30);

    // Step detection params
    step_height_ = this->declare_parameter<double>("step_height", 0.15);     // 15 cm target
    step_tol_    = this->declare_parameter<double>("step_tol", 0.05);        // ±5 cm
    percentile_  = this->declare_parameter<double>("floor_percentile", 0.10);// 10th percentile

    // Debounce
    consecutive_required_ = this->declare_parameter<int>("consecutive_required", 5);
    consecutive_count_ = 0;
    last_pub_ = false;

    pub_ = this->create_publisher<std_msgs::msg::Bool>("/stairs_detected", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StairsDetector::cloudCb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "StairsDetector listening on %s", cloud_topic_.c_str());
  }

private:
  static double percentile(std::vector<double>& v, double p) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    p = std::clamp(p, 0.0, 1.0);
    const size_t k = static_cast<size_t>(std::floor(p * (v.size() - 1)));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
  }

  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Expect xyz fields; UT LiDAR clouds normally include x/y/z float32
    if (msg->width * msg->height == 0) return;

    const int num_bins = std::max(1, static_cast<int>(std::ceil((x_max_ - x_min_) / bin_size_)));
    std::vector<std::vector<double>> z_bins(num_bins);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (x < x_min_ || x > x_max_) continue;
      if (y < y_min_ || y > y_max_) continue;
      if (z < z_min_ || z > z_max_) continue;

      const int b = static_cast<int>((x - x_min_) / bin_size_);
      if (b < 0 || b >= num_bins) continue;
      z_bins[b].push_back(static_cast<double>(z));
    }

    // Compute floor estimate per bin
    std::vector<double> floor_z(num_bins, std::numeric_limits<double>::quiet_NaN());
    std::vector<int> counts(num_bins, 0);

    for (int i = 0; i < num_bins; ++i) {
      counts[i] = static_cast<int>(z_bins[i].size());
      if (counts[i] < min_points_per_bin_) continue;
      floor_z[i] = percentile(z_bins[i], percentile_);
    }

    // Detect a step: floor_z jumps up ~15 cm between adjacent valid bins
    bool stairs_now = false;
    for (int i = 0; i < num_bins - 1; ++i) {
      if (!std::isfinite(floor_z[i]) || !std::isfinite(floor_z[i+1])) continue;
      const double dz = floor_z[i+1] - floor_z[i];
      if (std::abs(dz - step_height_) <= step_tol_) {
        stairs_now = true;
        break;
      }
    }

    // Debounce (consecutive frames)
    if (stairs_now) {
      consecutive_count_++;
    } else {
      consecutive_count_ = 0;
    }

    const bool stairs_debounced = (consecutive_count_ >= consecutive_required_);

    if (stairs_debounced != last_pub_) {
      std_msgs::msg::Bool out;
      out.data = stairs_debounced;
      pub_->publish(out);
      last_pub_ = stairs_debounced;

      RCLCPP_INFO(this->get_logger(), "stairs_detected=%s (count=%d)",
                  stairs_debounced ? "true" : "false", consecutive_count_);
    }
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

  // Params
  std::string cloud_topic_;
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
  double bin_size_;
  int min_points_per_bin_;
  double step_height_, step_tol_, percentile_;
  int consecutive_required_;

  // State
  int consecutive_count_;
  bool last_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StairsDetector>());
  rclcpp::shutdown();
  return 0;
}
