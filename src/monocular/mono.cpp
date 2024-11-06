#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"
#include "common.hpp"
#include <aruco_msgs/msg/marker_array.hpp>

void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Agora usamos a nova função que processa diretamente as tags ArUco
    // process_aruco_tags(msg);
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Criação do sistema SLAM
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    // Subscrição dos marcadores ArUco
    auto marker_sub = node->create_subscription<aruco_msgs::msg::MarkerArray>(
        "/aruco_marker_publisher/markers", 10, marker_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}