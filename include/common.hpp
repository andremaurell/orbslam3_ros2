#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sophus/se3.hpp>


#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Estruturas de dados globais
struct SLAMData {
    std::vector<int> marker_ids;
    std::vector<Sophus::SE3f> marker_poses;
};
extern SLAMData slam_data;

// Funções utilitárias
void process_aruco_tags(cv::Mat &frame, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                        const std::map<int, Sophus::SE3f> &known_markers);
void load_markers_from_json(const std::string &filename, std::map<int, Sophus::SE3f> &known_markers);
Eigen::Matrix4f calculate_camera_pose(
    const Sophus::SE3f &T_ar_world,
    const cv::Vec3d &rvec,
    const cv::Vec3d &tvec);
    cv::Mat convert_ros_image_to_cv(const sensor_msgs::msg::Image::SharedPtr msg);

// Classe SLAMRelocalizationNode
class SLAMRelocalizationNode : public rclcpp::Node {
public:
    SLAMRelocalizationNode();
    cv::Mat camera_matrix_, dist_coeffs_;
    std::map<int, Sophus::SE3f> known_markers_;

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void send_pose_to_slam(const Sophus::SE3f &pose);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

#endif // COMMON_HPP
