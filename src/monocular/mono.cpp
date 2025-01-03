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
#include <map>

// Função para carregar os parâmetros da câmera a partir de um arquivo YAML
void load_camera_parameters(const std::string &filename, cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
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
    dist_coeffs = (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, 0.0);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Uso: mono <vocabulário> <configuração>" << std::endl;
        return 1;
    }

    // Declarar as matrizes
    cv::Mat camera_matrix, dist_coeffs;
    std::map<int, Sophus::SE3f> known_markers;

    // Carregar parâmetros da câmera e marcadores conhecidos
    try {
        std::string camera_params = argv[2]; // Arquivo YAML de configuração passado como argumento
        std::string markers_json = "/home/lognav/colcon_ws/src/orbslam3_ros2/config/arucos_infos.json";

        load_camera_parameters(camera_params, camera_matrix, dist_coeffs);

        // Converta para o tipo desejado após carregar os dados
        camera_matrix.convertTo(camera_matrix, CV_32F);
        dist_coeffs.convertTo(dist_coeffs, CV_32F);

        // Carregar marcadores conhecidos
        load_markers_from_json(markers_json, known_markers);

    } catch (const std::exception &e) {
        std::cerr << "Erro ao carregar parâmetros: " << e.what() << std::endl;
        return 1;
    }

    // Inicialize o sistema ORB-SLAM3
    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, visualization);

    // Criação do nó principal do SLAM
    auto monocular_node = std::make_shared<MonocularSlamNode>(&SLAM);

    // Criação do nó de relocalização usando ArUcos
    auto slam_relocalization_node = std::make_shared<SLAMRelocalizationNode>();

    // Configuração de parâmetros do relocalization node
    slam_relocalization_node->camera_matrix_ = camera_matrix;
    slam_relocalization_node->dist_coeffs_ = dist_coeffs;
    slam_relocalization_node->known_markers_ = known_markers;

    RCLCPP_INFO(monocular_node->get_logger(), "Nó Monocular e Relocalização inicializados.");

    // Inicia o spin para ambos os nós
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(monocular_node);
    executor.add_node(slam_relocalization_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
