// StereoInertialMode.cpp  — ROS 2 + ORB-SLAM3 with message_filters (no custom mutexes)

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

#include "System.h"         // ORB-SLAM3
#include <sophus/se3.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::placeholders::_1;
using std::placeholders::_2;

class StereoInertialMode : public rclcpp::Node {
public:
  using Image    = sensor_msgs::msg::Image;
  using Imu      = sensor_msgs::msg::Imu;
  using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;
  using ImuPtr   = sensor_msgs::msg::Imu::ConstSharedPtr;

  StereoInertialMode()
  : rclcpp::Node("stereo_inertial_slam")
  {
    // ---------- Params
    declare_parameter<std::string>("left_image_topic",  "/cam0/image_raw");
    declare_parameter<std::string>("right_image_topic", "/cam1/image_raw");
    declare_parameter<std::string>("imu_topic",         "/imu0");
    declare_parameter<std::string>("odometry_topic",    "/orb_slam/odom");
    declare_parameter<std::string>("voc_file_arg",      "file_not_set");
    declare_parameter<std::string>("settings_file_path_arg", "file_not_set");
    declare_parameter<bool>("debug_sync", false);
    declare_parameter<double>("stereo_max_time_diff", 0.010); // 10 ms slop

    left_topic_    = get_parameter("left_image_topic").as_string();
    right_topic_   = get_parameter("right_image_topic").as_string();
    imu_topic_     = get_parameter("imu_topic").as_string();
    odom_topic_    = get_parameter("odometry_topic").as_string();
    voc_path_      = get_parameter("voc_file_arg").as_string();
    settings_path_ = get_parameter("settings_file_path_arg").as_string();
    debug_sync_    = get_parameter("debug_sync").as_bool();
    max_slop_      = get_parameter("stereo_max_time_diff").as_double();

    // ---------- ORB-SLAM3 init
    sensor_type_ = ORB_SLAM3::System::IMU_STEREO;
    slam_ = std::make_unique<ORB_SLAM3::System>(
      voc_path_.c_str(), settings_path_.c_str(), sensor_type_, false /* no viewer */
    );
    RCLCPP_INFO(get_logger(), "ORB-SLAM3 IMU_STEREO initialized");

    // ---------- Publisher
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS());

    // ---------- Heartbeat
    if (debug_sync_) {
      heartbeat_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StereoInertialMode::heartbeat, this)
      );
    }

    // IMPORTANT:
    // Do NOT construct message_filters subscribers in the constructor using shared_from_this();
    // instead, defer to a one-shot timer so the Node is fully owned by a shared_ptr.
    init_filters_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      std::bind(&StereoInertialMode::init_filters, this)
    );
  }

  ~StereoInertialMode() override {
    if (slam_) slam_->Shutdown();
  }

private:
  // ---------- State/params
  std::string left_topic_, right_topic_, imu_topic_, odom_topic_;
  std::string voc_path_, settings_path_;
  bool   debug_sync_{false};
  double max_slop_{0.010};
  double last_image_time_{std::numeric_limits<double>::quiet_NaN()};

  // ---------- ORB-SLAM3
  std::unique_ptr<ORB_SLAM3::System> slam_;
  ORB_SLAM3::System::eSensor sensor_type_;

  // ---------- ROS I/O
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr init_filters_timer_;

  // ---------- message_filters members
  // Note: these must be pointers and outlive the synchronizer
  std::shared_ptr<message_filters::Subscriber<Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<Image>> right_sub_;
  std::shared_ptr<message_filters::Subscriber<Imu>>   imu_sub_;
  std::shared_ptr<message_filters::Cache<Imu>>        imu_cache_;

  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  std::shared_ptr<message_filters::Synchronizer<ApproxPolicy>> sync_;

  // ---------- Utilities
  static inline double stampToSec(const rclcpp::Time &t) { return t.seconds(); }

  static cv::Mat toMono8(const ImagePtr &msg, const rclcpp::Logger &logger) {
    try {
      if (msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "mono8") {
        return cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
      }
      // common color encodings -> gray
      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        auto c = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat g; cv::cvtColor(c, g, cv::COLOR_BGR2GRAY); return g;
      }
      if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
        auto c = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        cv::Mat g; cv::cvtColor(c, g, cv::COLOR_RGB2GRAY); return g;
      }
      // fallback: ask cv_bridge for mono8
      return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
      return {};
    }
  }

  static ORB_SLAM3::IMU::Point toImuPoint(const ImuPtr &m) {
    const double t = stampToSec(rclcpp::Time(m->header.stamp));
    cv::Point3f acc(m->linear_acceleration.x, m->linear_acceleration.y, m->linear_acceleration.z);
    cv::Point3f gyr(m->angular_velocity.x,    m->angular_velocity.y,    m->angular_velocity.z);
    return ORB_SLAM3::IMU::Point(acc, gyr, t);
  }

  void init_filters() {
    // Construct RELIABLE profiles to match rosbag2_player
    rmw_qos_profile_t reliable_sensor = rmw_qos_profile_sensor_data;
    reliable_sensor.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    reliable_sensor.depth = 50;  // small history to survive slight delays

    auto node = shared_from_this(); // safe here (not in ctor)

    left_sub_  = std::make_shared<message_filters::Subscriber<Image>>(node,  left_topic_,  reliable_sensor);
    right_sub_ = std::make_shared<message_filters::Subscriber<Image>>(node,  right_topic_, reliable_sensor);
    imu_sub_   = std::make_shared<message_filters::Subscriber<Imu>>(node,    imu_topic_,   reliable_sensor);

    imu_cache_ = std::make_shared<message_filters::Cache<Imu>>(*imu_sub_, 4000 /*max elems*/);
    // (optional) you can set a rough cache time range; Cache in ROS2 primarily uses size

    // Stereo sync (ApproximateTime with configurable slop)
    ApproxPolicy policy(10 /*queue size*/);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_slop_));
    sync_ = std::make_shared<message_filters::Synchronizer<ApproxPolicy>>(policy, *left_sub_, *right_sub_);
    sync_->registerCallback(std::bind(&StereoInertialMode::stereoCallback, this, _1, _2));

    if (debug_sync_) {
      RCLCPP_INFO(get_logger(), "message_filters initialized (RELIABLE QoS, slop=%.3f ms)",
                  max_slop_ * 1e3);
    }

    // Destroy the one-shot timer so we don’t re-init
    init_filters_timer_.reset();
  }

  void heartbeat() {
    // We can't peek sizes from message_filters directly. Just emit a soft heartbeat.
    RCLCPP_INFO(get_logger(), "[HEARTBEAT] running; slop=%.3f ms", max_slop_ * 1e3);
  }

  void stereoCallback(const ImagePtr &left, const ImagePtr &right) {
    const double tL = stampToSec(rclcpp::Time(left->header.stamp));
    const double tR = stampToSec(rclcpp::Time(right->header.stamp));

    if (debug_sync_) {
      RCLCPP_INFO(get_logger(),
        "[SYNC] L=%.6f  R=%.6f  Δ=%.3f ms",
        tL, tR, std::fabs(tL - tR) * 1e3
      );
    }

    // Gather IMU between (last_image_time_, tL]; if first time, just use up-to-tL
    const double t_prev = std::isfinite(last_image_time_) ? last_image_time_ : (tL - 1.0);
    last_image_time_ = tL;

    std::vector<ImuPtr> imu_msgs;
    if (imu_cache_) {
      // Cache returns a vector of shared_ptr between two stamps (inclusive)
      const rclcpp::Time t_begin = rclcpp::Time::from_seconds(t_prev);
      const rclcpp::Time t_end   = rclcpp::Time::from_seconds(tL);
      imu_cache_->getInterval(t_begin, t_end, imu_msgs);
    }

    if (imu_msgs.empty() && debug_sync_) {
      RCLCPP_WARN(get_logger(), "No IMU in (%.6f, %.6f]", t_prev, tL);
    }

    // Convert images
    cv::Mat imL = toMono8(left,  get_logger());
    cv::Mat imR = toMono8(right, get_logger());
    if (imL.empty() || imR.empty()) {
      if (debug_sync_) RCLCPP_WARN(get_logger(), "Empty cv::Mat, skipping stereo pair");
      return;
    }

    // Convert IMU to ORB-SLAM3 format
    std::vector<ORB_SLAM3::IMU::Point> vImu;
    vImu.reserve(imu_msgs.size());
    for (const auto &m : imu_msgs) vImu.emplace_back(toImuPoint(m));

    // Track
    Sophus::SE3f Tcw = slam_->TrackStereo(imL, imR, tL, vImu);
    Sophus::SE3f Twc = Tcw.inverse();

    // Publish odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = left->header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id  = "camera_link";

    odom.pose.pose.position.x = Twc.translation().x();
    odom.pose.pose.position.y = Twc.translation().y();
    odom.pose.pose.position.z = Twc.translation().z();

    Eigen::Quaternionf q(Twc.rotationMatrix());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    for (double &c : odom.pose.covariance) c = 0.0;

    odom_pub_->publish(odom);
  }
};

// --------- main ----------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoInertialMode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
