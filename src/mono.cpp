#include <memory>
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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class MonoMode : public rclcpp::Node{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
        std::unique_ptr<ORB_SLAM3::System> slam_;
        ORB_SLAM3::System::eSensor sensor_type_ = ORB_SLAM3::System::MONOCULAR;

        std::string image_topic, odom_topic_;
        std::string voc_path_, settings_path_;
        bool   debug_info_{false};

        static cv::Mat toMono8(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const rclcpp::Logger &logger) {
            try {
            if (msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "mono8")
                return cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
            if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
                auto c = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
                cv::Mat g; cv::cvtColor(c, g, cv::COLOR_BGR2GRAY); return g;
            }
            if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
                auto c = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
                cv::Mat g; cv::cvtColor(c, g, cv::COLOR_RGB2GRAY); return g;
            }
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
            } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
            return {};
            }
        }

        void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
            cv::Mat im = toMono8(msg, get_logger());

            if (im.empty()){
                RCLCPP_WARN(get_logger(), "Empty image, skipping.");
                    return;
            }

            Sophus::SE3f Tcw;
            double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
            try {
                Tcw = slam_->TrackMonocular(im,t);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "TrackStereo exception: %s", e.what());
                return;
            }

            Eigen::Matrix4f M = Tcw.matrix();
            if (!M.allFinite()) { RCLCPP_WARN(get_logger(), "NaN Tcw, skipping."); return; }
            Eigen::Matrix3f Rcw = Tcw.rotationMatrix();
            float detR = Rcw.determinant()
            ;
            if (!std::isfinite(detR) || std::abs(detR - 1.0f) > 0.2f || Rcw.isZero(0)) {
                RCLCPP_WARN(get_logger(), "Bad rotation (det=%.3f), skipping.", detR);
                return;
            }


            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Quaternionf q(Twc.rotationMatrix());

            nav_msgs::msg::Odometry odom;
            odom.header.stamp    = msg->header.stamp;
            odom.header.frame_id = "map";
            odom.child_frame_id  = "camera_link";
            odom.pose.pose.position.x = Twc.translation().x();
            odom.pose.pose.position.y = Twc.translation().y();
            odom.pose.pose.position.z = Twc.translation().z();
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            for (double &c : odom.pose.covariance) c = 0.0;
            odometry_pub_->publish(odom);
        }
    public:
        ~MonoMode() override {
            if (slam_) slam_->Shutdown();
        }
        MonoMode() : rclcpp::Node("mono_slam"){
            declare_parameter<std::string>("image_topic",  "/cam0/image_raw");
            declare_parameter<std::string>("odometry_topic",    "/orb_slam/odom");
            declare_parameter<std::string>("voc_file_arg",      "file_not_set");
            declare_parameter<std::string>("settings_file_path_arg", "file_not_set");
            declare_parameter<bool>("debug_info", false);
            
            image_topic     = get_parameter("image_topic").as_string();
            odom_topic_     = get_parameter("odometry_topic").as_string();
            voc_path_       = get_parameter("voc_file_arg").as_string();
            settings_path_  = get_parameter("settings_file_path_arg").as_string();
            debug_info_     = get_parameter("debug_info").as_bool();

            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&MonoMode::image_callback, this, std::placeholders::_1)
            );

            odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                odom_topic_,
                rclcpp::SensorDataQoS()
            );

            slam_ = std::make_unique<ORB_SLAM3::System>(voc_path_, settings_path_, sensor_type_, debug_info_);

            RCLCPP_INFO(get_logger(), "Mono Mode started successfully!");
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonoMode>();
    if (!rclcpp::ok()) return 1;
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}