#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tbb/concurrent_queue.h>


#include "System.h"
#include <sophus/se3.hpp>

using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;
using ImuPtr   = sensor_msgs::msg::Imu::ConstSharedPtr;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

struct StereoBundle {
  double t;
  cv::Mat left;
  cv::Mat right;
};

struct ImuSample {
  double t;
  Eigen::Vector3f gyro;
  Eigen::Vector3f accel;
};

class StereoInertialNode : public rclcpp::Node {
private:
  // ---------------- Queues ----------------
  tbb::concurrent_queue<ImuSample>     imu_queue;
  tbb::concurrent_queue<StereoBundle>  stereo_queue;

  std::atomic<bool> running{true};

  // ---------------- ORB-SLAM3 ----------------
  std::unique_ptr<ORB_SLAM3::System> slam_;
  ORB_SLAM3::System::eSensor sensor_type_;

  // ---------------- ROS IO ----------------
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  double imu_time_offset_;

  static inline double toSec(const rclcpp::Time &t) { return t.nanoseconds() * 1e-9; }

  static cv::Mat toGray(const ImagePtr &msg) {
    cv::Mat img = cv_bridge::toCvShare(msg)->image;
    
    // If mono16 → convert to mono8 by right shift
    if (img.type() == CV_16UC1) {
      cv::Mat img8;
      img.convertTo(img8, CV_8UC1, 1.0/256.0);  // scale down 16 -> 8 bit
      return img8;
    }

    // If already mono8
    if (img.type() == CV_8UC1) 
      return img;

    // If color → convert
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }

  // ---------------- Callbacks: only queue push, no work ----------------
  void imu_cb(ImuPtr msg) {
    ImuSample s;
    s.t = toSec(msg->header.stamp) + imu_time_offset_;
    s.gyro  = Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    s.accel = Eigen::Vector3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    imu_queue.push(s);     // non-blocking, drop if overflow
  }

  void stereo_cb(const ImagePtr &l, const ImagePtr &r) {
    StereoBundle b;
    b.t = toSec(l->header.stamp);
    b.left  = toGray(l);
    b.right = toGray(r);

    stereo_queue.push(b);      // blocking = natural pacing
  }

  // ---------------- Worker thread ----------------
  void processingThread() {
    double last_t = 0.0;

    while (running) {
      StereoBundle frame;
      if (!stereo_queue.try_pop(frame)) continue;

      if (frame.left.empty() || frame.right.empty()) continue;

      // drain IMU up to current stereo timestamp
      std::vector<ORB_SLAM3::IMU::Point> imu_vec;
      ImuSample s;
      while (imu_queue.try_pop(s)) {
        if (s.t > frame.t) { imu_queue.push(s); break; }
        imu_vec.emplace_back(
          s.accel.x(), s.accel.y(), s.accel.z(),
          s.gyro.x(),  s.gyro.y(),  s.gyro.z(),
          s.t);
      }

      // SLAM call (single threaded, stable)
      Sophus::SE3f Tcw;
      try {
        Tcw = slam_->TrackStereo(frame.left, frame.right, frame.t, imu_vec);
      } catch (...) { continue; }

      if (!Tcw.matrix().allFinite()) continue;

      // Publish odometry
      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Quaternionf q(Twc.rotationMatrix());

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = rclcpp::Time(frame.t * 1e9);
      odom.header.frame_id = "map";
      odom.child_frame_id  = "camera_link";
      odom.pose.pose.position.x = Twc.translation().x();
      odom.pose.pose.position.y = Twc.translation().y();
      odom.pose.pose.position.z = Twc.translation().z();
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom_pub_->publish(odom);
    }
  }

public:
  ~StereoInertialNode() override { running = false; slam_->Shutdown(); }

  StereoInertialNode()
   : Node("stereo_inertial_slam"), left_sub_(this, ""), right_sub_(this, "") {

    // ------------ params ------------
    declare_parameter<std::string>("left_image_topic", "/cam0/image_raw");
    declare_parameter<std::string>("right_image_topic","/cam1/image_raw");
    declare_parameter<std::string>("imu_topic",        "/imu0");
    declare_parameter<std::string>("odometry_topic",   "/odom");
    declare_parameter<std::string>("voc_file_arg",              "ORBvoc.bin");
    declare_parameter<std::string>("settings_file_path_arg",         "stereo.yaml");
    declare_parameter<double>("imu_time_offset",       0.0);

    imu_time_offset_ = get_parameter("imu_time_offset").as_double();
    auto voc = get_parameter("voc_file_arg").as_string();
    auto set = get_parameter("settings_file_path_arg").as_string();
    auto lt  = get_parameter("left_image_topic").as_string();
    auto rt  = get_parameter("right_image_topic").as_string();
    auto it  = get_parameter("imu_topic").as_string();
    auto odt = get_parameter("odometry_topic").as_string();

    // ------------ queue capacities (critical) ------------
    // ------------ SLAM init ------------
    sensor_type_ = ORB_SLAM3::System::IMU_STEREO;
    slam_ = std::make_unique<ORB_SLAM3::System>(voc.c_str(), set.c_str(), sensor_type_, false);

    // ------------ ROS IO ------------
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        it, rclcpp::SensorDataQoS(), std::bind(&StereoInertialNode::imu_cb, this, std::placeholders::_1));

    left_sub_.subscribe(this, lt);
    right_sub_.subscribe(this, rt);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), left_sub_, right_sub_);
    sync_->registerCallback(std::bind(&StereoInertialNode::stereo_cb, this, std::placeholders::_1, std::placeholders::_2));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odt, rclcpp::SensorDataQoS());

    // ------------ worker thread ------------
    std::thread(&StereoInertialNode::processingThread, this).detach();

    RCLCPP_INFO(get_logger(), "ORB-SLAM3 stereo-inertial with TBB queues started.");
  }
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<StereoInertialNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(n);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
