#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"
#include "common.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <string>
#include <stdexcept>

// Função para carregar os parâmetros da câmera a partir de um arquivo YAML
void load_camera_parameters(const std::string &filename, cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Falha ao abrir o arquivo de parâmetros da câmera: " + filename);
    }

    double fx, fy, cx, cy, k1, k2, p1, p2;

    // Leia os parâmetros diretamente do arquivo YAML
    fx = (double)fs["Camera.fx"];
    fy = (double)fs["Camera.fy"];
    cx = (double)fs["Camera.cx"];
    cy = (double)fs["Camera.cy"];
    k1 = (double)fs["Camera.k1"];
    k2 = (double)fs["Camera.k2"];
    p1 = (double)fs["Camera.p1"];
    p2 = (double)fs["Camera.p2"];

    // Construa a matriz da câmera
    camera_matrix = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    // Construa os coeficientes de distorção
    dist_coeffs = (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, 0.0);}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Declarar as matrizes
    cv::Mat camera_matrix, dist_coeffs;

    // Carregar parâmetros da câmera do arquivo YAML
    try {
        load_camera_parameters("/home/lognav/colcon_ws/src/orbslam3_ros2/config/monocular/EuRoC.yaml", camera_matrix, dist_coeffs);

        // Converta para o tipo desejado após carregar os dados
        camera_matrix.convertTo(camera_matrix, CV_32F);
        dist_coeffs.convertTo(dist_coeffs, CV_32F);
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    // Criação do sistema SLAM
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);

    // Assinatura para callback de imagem
    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/freedom_vehicle/camera/image_raw", 10,
        [&](const sensor_msgs::msg::Image::SharedPtr msg) {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            process_aruco_tags(frame, camera_matrix, dist_coeffs);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

