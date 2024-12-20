#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#include "common.hpp"

// Definição da variável global slam_data
SLAMData slam_data;

// Função para processar os marcadores ArUco
// Processar ArUcos
void process_aruco_tags(cv::Mat &frame, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                        const std::map<int, Sophus::SE3f> &known_markers) {
    if (frame.empty()) {
        std::cerr << "Error: Frame is empty!" << std::endl;
        return;
    }

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        throw std::runtime_error("Camera parameters are not loaded properly.");
    }

    cv::Mat cam_mat, dist_coeffs_mat;
    camera_matrix.convertTo(cam_mat, CV_64F);
    dist_coeffs.convertTo(dist_coeffs_mat, CV_64F);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;

        for (size_t i = 0; i < ids.size(); ++i) {
            int marker_id = ids[i];

            if (known_markers.count(marker_id)) {
                // Tamanho do marcador em metros
                float marker_size = 0.071;  // Substitua pelo tamanho correto dos marcadores em metros
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cam_mat, dist_coeffs_mat, rvecs, tvecs);

                // Normalizar tvec para metros, se necessário
                tvecs[i][0] /= 100.0;
                tvecs[i][1] /= 100.0;
                tvecs[i][2] /= 100.0;

                std::cout << "ArUco ID: " << marker_id
                          << "\nrvec: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2]
                          << "]\ntvec: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;

                // Calcular a posição da câmera no mundo
                Eigen::Matrix4f T_cam_world = calculate_camera_pose(
                    known_markers.at(marker_id), rvecs[i], tvecs[i]);

                // Exibir posição calculada da câmera
                std::cout << "Detected ArUco Marker ID: " << marker_id
                          << " Camera Position in World: ["
                          << T_cam_world(0, 3) << ", "
                          << T_cam_world(1, 3) << ", "
                          << T_cam_world(2, 3) << "]" << std::endl;

                // Desenhar eixo com tamanho reduzido
                float axis_length = marker_size * 0.1;  // Reduzir o tamanho dos eixos
                cv::aruco::drawAxis(frame, cam_mat, dist_coeffs_mat, rvecs[i], tvecs[i], axis_length);
            } else {
                std::cerr << "Marker ID " << marker_id << " not found in known markers." << std::endl;
            }
        }
    }

    cv::imshow("ArUco Detection", frame);
    cv::waitKey(1);
}


// Função para carregar marcadores do JSON
void load_markers_from_json(const std::string &filename, std::map<int, Sophus::SE3f> &known_markers) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open JSON file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;

    for (const auto &marker : json_data) {
        int id = marker.at("id").get<int>();
        Eigen::Vector3f position(
            marker.at("pos_x").get<float>(),
            marker.at("pos_y").get<float>(),
            marker.at("pos_z").get<float>());

        float yaw = marker.at("yaw").get<float>();
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        Sophus::SE3f pose(rotation, position);
        known_markers[id] = pose;

        std::cout << "Loaded Marker ID: " << id
                  << " Position: (" << position.x() << ", " << position.y() << ", " << position.z() << ")"
                  << " Yaw: " << yaw << std::endl;
    }

    std::cout << "Loaded " << known_markers.size() << " markers from JSON." << std::endl;
}

// Função para calcular a posição da câmera no mundo
Eigen::Matrix4f calculate_camera_pose(
    const Sophus::SE3f &T_ar_world,
    const cv::Vec3d &rvec,
    const cv::Vec3d &tvec) {
    // Converter rvec para matriz de rotação
    cv::Mat R_cam;
    cv::Rodrigues(rvec, R_cam);
    Eigen::Matrix3f R_ar_cam;
    cv::cv2eigen(R_cam, R_ar_cam);

    // Vetor de translação
    Eigen::Vector3f t_ar_cam(tvec[0], tvec[1], tvec[2]);

    // Matriz de transformação do ArUco para a câmera
    Eigen::Matrix4f T_ar_cam = Eigen::Matrix4f::Identity();
    T_ar_cam.block<3, 3>(0, 0) = R_ar_cam;
    T_ar_cam.block<3, 1>(0, 3) = t_ar_cam;

    // Inversão da transformação (câmera em relação ao ArUco)
    Eigen::Matrix4f T_cam_ar = T_ar_cam.inverse();

    // Matriz de transformação da câmera para o mundo
    Eigen::Matrix4f T_cam_world = T_ar_world.matrix() * T_cam_ar;

    return T_cam_world;
}

