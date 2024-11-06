#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"
#include "System.h"
#include "common.hpp"
#include <aruco_msgs/msg/marker_array.hpp>

// Callback para processar as tags ArUco
 void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
 {
     // Processa as tags ArUco utilizando a função previamente implementada
     // process_aruco_tags(msg);
 }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }
    auto node = std::make_shared<rclcpp::Node>("run_slam");

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    std::shared_ptr<StereoSlamNode> slam_ros;
    slam_ros = std::make_shared<StereoSlamNode>(&pSLAM, node.get(), argv[2], argv[3]);
    std::cout << "============================ " << std::endl;

    // Subscrição dos marcadores ArUco (assim como no Monocular)
    // auto marker_sub = node->create_subscription<aruco_msgs::msg::MarkerArray>(
    //     "/aruco_marker_publisher/markers", 10, marker_callback);

    rclcpp::spin(slam_ros->node_->get_node_base_interface());
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

