#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <ctime>
#include <filesystem>
#include <numeric>
#include <deque>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

#include "System.h"
#include <sophus/se3.hpp>

using std::placeholders::_1;
using Image    = sensor_msgs::msg::Image;
using Imu      = sensor_msgs::msg::Imu;
using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;
using ImuPtr   = sensor_msgs::msg::Imu::ConstSharedPtr;

class IMUBuffer {
private:
  mutable std::mutex mutex_;
  std::deque<ORB_SLAM3::IMU::Point> buffer_;
  const double max_duration_;

public:
  explicit IMUBuffer(double max_duration_sec = 5.0)
      : max_duration_(max_duration_sec) {}

  void push(const ImuPtr& msg, double time) {
    if (!std::isfinite(msg->linear_acceleration.x) ||
        !std::isfinite(msg->linear_acceleration.y) ||
        !std::isfinite(msg->linear_acceleration.z) ||
        !std::isfinite(msg->angular_velocity.x)   ||
        !std::isfinite(msg->angular_velocity.y)   ||
        !std::isfinite(msg->angular_velocity.z))
      return;

    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.emplace_back(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z, time);

    const double cutoff = time - max_duration_;
    while (!buffer_.empty() && buffer_.front().t < cutoff)
      buffer_.pop_front();
  }

  std::vector<ORB_SLAM3::IMU::Point> extractBetween(double t0, double t1) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ORB_SLAM3::IMU::Point> selected;

    if (buffer_.empty() || t1 <= t0)
      return selected;

    for (const auto &p : buffer_) {
      if (p.t >= t0 && p.t < t1)
        selected.push_back(p);
    }
    return selected;
  }
};

class StereoInertialMode : public rclcpp::Node {
private:
  std::string image_topic, imu_topic_, odom_topic_;
  std::string voc_path_, settings_path_;
  std::string debug_log_path_;
  bool debug_info_{false};
  std::ofstream debug_log_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  ORB_SLAM3::System::eSensor sensor_type_;

  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<Image>::SharedPtr mono_sub_;

  std::shared_ptr<IMUBuffer> buffer_;

  // ------------------------------
  // IMU freq between image frames
  // ------------------------------
  double last_image_time_{0.0};
  bool first_frame_{true}, first_imu_{true};

  double imu_time_offset_{0.0}, last_imu_time_{0.0};

  // Acceleration debug (unchanged)
  std::deque<double> accel_norms_;

  static inline double stampToSec(const rclcpp::Time &t) {
    return static_cast<double>(t.nanoseconds()) * 1e-9;
  }

  void logDebug(const std::string &msg) {
    if (debug_info_ && debug_log_.is_open())
      debug_log_ << msg << std::endl;
  }

  static cv::Mat toMono8(const ImagePtr &msg, const rclcpp::Logger &logger) {
    try {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        return cv_bridge::toCvShare(msg, "mono8")->image.clone();

      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        auto c = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat g; cv::cvtColor(c, g, cv::COLOR_BGR2GRAY);
        return g;
      }

      if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        auto c = cv_bridge::toCvShare(msg, "rgb8")->image;
        cv::Mat g; cv::cvtColor(c, g, cv::COLOR_RGB2GRAY);
        return g;
      }

      return cv_bridge::toCvCopy(msg, "mono8")->image;

    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
      return {};
    }
  }

  // ---------------------------------
  // IMU CALLBACK
  // ---------------------------------
  void imu_callback(ImuPtr msg) {
    const double t_current = stampToSec(msg->header.stamp) + imu_time_offset_;

    buffer_->push(msg, t_current);

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    accel_norms_.push_back(std::sqrt(ax*ax + ay*ay + az*az));

  }

  // ---------------------------------
  // IMAGE CALLBACK — MAIN LOOP
  // ---------------------------------
  void mono_callback(const ImagePtr& img) {
    const double t_current = stampToSec(img->header.stamp);

    // initialize
    if (first_frame_) {
      last_image_time_ = t_current;
      first_frame_ = false;
    }

    // calculate IMU frequency from last frame
    double dt = t_current - last_image_time_;
    double imu_freq = 0.0;


    // debug info
   

    // extract IMU for ORB-SLAM
    std::vector<ORB_SLAM3::IMU::Point> imu_vec =
      buffer_->extractBetween(last_image_time_, t_current);

    

    last_image_time_ = t_current;

    cv::Mat im = toMono8(img, get_logger());
    if (im.empty()) return;

    Sophus::SE3f Tcw;
    try {
      Tcw = slam_->TrackMonocular(im, t_current, imu_vec);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "TrackMonocular exception: %s", e.what());
      return;
    }

    Eigen::Matrix4f M = Tcw.matrix();
    if (!M.allFinite()) return;

    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = img->header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "camera_link";

    odom.pose.pose.position.x = Twc.translation().x();
    odom.pose.pose.position.y = Twc.translation().y();
    odom.pose.pose.position.z = Twc.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    for (double &c : odom.pose.covariance)
      c = 0.0;

    odom_pub_->publish(odom);
  }

public:

  ~StereoInertialMode() override {
    if (debug_log_.is_open())
      debug_log_.close();
    if (slam_)
      slam_->Shutdown();
  }

  StereoInertialMode() : rclcpp::Node("stereo_inertial_slam") {
    declare_parameter<std::string>("image_topic", "/cam0/image_raw");
    declare_parameter<std::string>("imu_topic", "/imu0");
    declare_parameter<std::string>("odometry_topic", "/orb_slam/odom");
    declare_parameter<std::string>("voc_file_arg", "file_not_set");
    declare_parameter<std::string>("settings_file_path_arg", "file_not_set");
    declare_parameter<std::string>("debug_log_path", "");
    declare_parameter<bool>("debug_info", false);
    declare_parameter<double>("imu_time_offset", 0.0);

    image_topic     = get_parameter("image_topic").as_string();
    imu_topic_      = get_parameter("imu_topic").as_string();
    odom_topic_     = get_parameter("odometry_topic").as_string();
    voc_path_       = get_parameter("voc_file_arg").as_string();
    settings_path_  = get_parameter("settings_file_path_arg").as_string();
    debug_log_path_ = get_parameter("debug_log_path").as_string();
    debug_info_     = get_parameter("debug_info").as_bool();
    imu_time_offset_ = get_parameter("imu_time_offset").as_double();

    if (debug_info_) {
      if (debug_log_path_.empty() || !std::filesystem::exists(debug_log_path_)) {
        rclcpp::shutdown();
        return;
      }
      auto now = std::chrono::system_clock::now();
      std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm = *std::localtime(&t);
      char filename[128];
      std::strftime(filename, sizeof(filename),
                    "debug_%Y%m%d_%H%M%S.log", &tm);

      std::filesystem::path fp =
        std::filesystem::path(debug_log_path_) / filename;

      debug_log_.open(fp, std::ios::out | std::ios::app);
      if (!debug_log_.is_open()) {
        rclcpp::shutdown();
        return;
      }
    }

    buffer_ = std::make_shared<IMUBuffer>(50.0);

    imu_sub_ = this->create_subscription<Imu>(
        imu_topic_,
        rclcpp::QoS(rclcpp::KeepLast(400))
            .reliable()
            .durability_volatile(),
        std::bind(&StereoInertialMode::imu_callback, this, _1)
    );

    mono_sub_ = this->create_subscription<Image>(
        image_topic,
        rclcpp::QoS(rclcpp::KeepLast(200))
            .reliable()
            .durability_volatile(),
        std::bind(&StereoInertialMode::mono_callback, this, _1)
    );


    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS());

    sensor_type_ = ORB_SLAM3::System::IMU_MONOCULAR;

    slam_ = std::make_unique<ORB_SLAM3::System>(
      voc_path_.c_str(), settings_path_.c_str(),
      sensor_type_, debug_info_);

    RCLCPP_INFO(get_logger(),
                "Mono-Inertial SLAM node with per-frame IMU freq initialized.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoInertialMode>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
