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

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <image_transport/image_transport.hpp>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/msg/odometry.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Transformation process
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform.h>

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"

// JSON library
#include <nlohmann/json.hpp>
using json = nlohmann::json;

extern ORB_SLAM3::System *pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;

extern double roll, pitch, yaw;
extern bool publish_static_transform;
extern std::string world_frame_id, cam_frame_id, imu_frame_id, map_frame_id;

extern image_transport::Publisher tracking_img_pub;
extern ros::Publisher pose_pub, odom_pub, kf_markers_pub;
extern ros::Publisher tracked_mappoints_pub, all_mappoints_pub;

extern rviz_visual_tools::RvizVisualToolsPtr wall_visual_tools;

struct MapPointStruct {
    Eigen::Vector3f coordinates;
    int cluster_id;

    MapPointStruct(Eigen::Vector3f coords) : coordinates(coords), cluster_id(-1) {}
};

// Data structure to hold combined ArUco marker and keypoint data
struct SLAMData {
    std::vector<int> marker_ids;
    std::vector<Sophus::SE3f> marker_poses;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

extern SLAMData slam_data;

// Functions
void setup_services(ros::NodeHandle &, std::string);
void publish_topics(ros::Time, Eigen::Vector3f = Eigen::Vector3f::Zero());
void setup_publishers(ros::NodeHandle &, image_transport::ImageTransport &, std::string);

void publish_tracking_img(cv::Mat, ros::Time);
void publish_camera_pose(Sophus::SE3f, ros::Time);
void publish_static_tf_transform(std::string, std::string, ros::Time);
void publish_kf_markers(std::vector<Sophus::SE3f>, ros::Time);
void publish_tf_transform(Sophus::SE3f, std::string, std::string, ros::Time);
void publish_all_points(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);
void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);
void publish_body_odom(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, ros::Time);

cv::Mat SE3f_to_cvMat(Sophus::SE3f);
inline geometry_msgs::msg::Transform SE3f_to_transform(Sophus::SE3f pose);

sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *>, ros::Time);

// Functions to process ArUco markers and calculate pose
void process_aruco_tags(cv::Mat &frame, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                        const std::map<int, Sophus::SE3f> &known_markers);
void load_markers_from_json(const std::string &filename, std::map<int, Sophus::SE3f> &known_markers);
Eigen::Matrix4f calculate_camera_pose(const Sophus::SE3f &T_ar_world, const cv::Vec3d &rvec, const cv::Vec3d &tvec);

#endif // COMMON_HPP
