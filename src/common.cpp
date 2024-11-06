#include "common.hpp"
#include <aruco_msgs/msg/marker_array.hpp>

// Global SLAMData instance
SLAMData slam_data;

// void process_aruco_tags(const aruco_msgs::msg::MarkerArray::SharedPtr marker_array)
// {
//     slam_data.marker_ids.clear();
//     slam_data.marker_poses.clear();

//     for (const auto &marker : marker_array->markers) {
//         int marker_id = marker.id;
//         auto marker_position = marker.pose.pose.position;
//         auto marker_orientation = marker.pose.pose.orientation;

//         Eigen::Vector3f marker_translation(marker_position.x, marker_position.y, marker_position.z);
//         Eigen::Quaternionf marker_quaternion(marker_orientation.w, marker_orientation.x,
//                                              marker_orientation.y, marker_orientation.z);
//         Sophus::SE3f normalized_pose(marker_quaternion, marker_translation);

//         slam_data.marker_ids.push_back(marker_id);
//         slam_data.marker_poses.push_back(normalized_pose);
//     }
// }

// Other functions would be implemented here as needed, e.g., setup_services, publish_topics, etc.
