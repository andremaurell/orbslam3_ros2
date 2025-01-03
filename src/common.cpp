#include "common.hpp"
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <sophus/se3.hpp>


// Definição da variável global slam_data
SLAMData slam_data;

// Função para processar os marcadores ArUco
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

        slam_data.marker_ids.clear();
        slam_data.marker_poses.clear();

        for (size_t i = 0; i < ids.size(); ++i) {
            int marker_id = ids[i];

            if (known_markers.count(marker_id)) {
                float marker_size = 0.071;  // Tamanho do marcador em metros
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cam_mat, dist_coeffs_mat, rvecs, tvecs);

                tvecs[i][0] /= 100.0;
                tvecs[i][1] /= 100.0;
                tvecs[i][2] /= 100.0;

                std::cout << "ArUco ID: " << marker_id
                          << "\nrvec: [" << rvecs[i][0] << ", " << rvecs[i][1] << ", " << rvecs[i][2]
                          << "]\ntvec: [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "]" << std::endl;

                Eigen::Matrix4f T_cam_world = calculate_camera_pose(
                    known_markers.at(marker_id), rvecs[i], tvecs[i]);

                Sophus::SE3f marker_pose(Eigen::Matrix3f(T_cam_world.block<3, 3>(0, 0)),
                                         Eigen::Vector3f(T_cam_world.block<3, 1>(0, 3)));

                slam_data.marker_ids.push_back(marker_id);
                slam_data.marker_poses.push_back(marker_pose);

                std::cout << "Atualizado slam_data - Marker ID: " << marker_id
                          << " Pose: [" << T_cam_world(0, 3) << ", " << T_cam_world(1, 3) << ", " << T_cam_world(2, 3) << "]" << std::endl;

                float axis_length = marker_size * 0.1;  // Tamanho reduzido do eixo
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
    cv::Mat R_cam;
    cv::Rodrigues(rvec, R_cam);
    Eigen::Matrix3f R_ar_cam;
    cv::cv2eigen(R_cam, R_ar_cam);

    Eigen::Vector3f t_ar_cam(tvec[0], tvec[1], tvec[2]);

    Eigen::Matrix4f T_ar_cam = Eigen::Matrix4f::Identity();
    T_ar_cam.block<3, 3>(0, 0) = R_ar_cam;
    T_ar_cam.block<3, 1>(0, 3) = t_ar_cam;

    Eigen::Matrix4f T_cam_ar = T_ar_cam.inverse();
    Eigen::Matrix4f T_cam_world = T_ar_world.matrix() * T_cam_ar;

    return T_cam_world;
}

cv::Mat convert_ros_image_to_cv(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        return cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const std::exception &e) {
        throw std::runtime_error("Erro ao converter imagem: " + std::string(e.what()));
    }
}

// Implementação de SLAMRelocalizationNode
SLAMRelocalizationNode::SLAMRelocalizationNode()
    : Node("slam_relocalization_node") {
    RCLCPP_INFO(this->get_logger(), "SLAMRelocalizationNode iniciado.");

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/freedom_vehicle/camera/image_raw", 10,
        std::bind(&SLAMRelocalizationNode::imageCallback, this, std::placeholders::_1));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/camera_pose", 10);

    try {
        load_markers_from_json("/home/lognav/colcon_ws/src/orbslam3_ros2/config/arucos_infos.json", known_markers_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Erro ao carregar marcadores: %s", e.what());
    }
}

void SLAMRelocalizationNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Callback de imagem chamado.");

    cv::Mat frame = convert_ros_image_to_cv(msg);

    process_aruco_tags(frame, camera_matrix_, dist_coeffs_, known_markers_);

    if (slam_data.marker_ids.empty()) {
        RCLCPP_WARN(this->get_logger(), "Nenhum marcador detectado na imagem.");
        return;
    }

    bool relocalized = false;
    for (size_t i = 0; i < slam_data.marker_ids.size(); ++i) {
        int id = slam_data.marker_ids[i];
        if (known_markers_.count(id)) {
            Sophus::SE3f marker_pose = slam_data.marker_poses[i];

            Eigen::Vector3f camera_position = marker_pose.translation();
            RCLCPP_INFO(this->get_logger(), "Relocalizado com ArUco ID: %d", id);
            RCLCPP_INFO(this->get_logger(), "Posição da câmera no mundo: [%.3f, %.3f, %.3f]",
                        camera_position[0], camera_position[1], camera_position[2]);

            send_pose_to_slam(marker_pose);
            relocalized = true;
            break;
        } else {
            RCLCPP_WARN(this->get_logger(), "Marcador ID %d não é conhecido.", id);
        }
    }

    if (!relocalized) {
        RCLCPP_WARN(this->get_logger(), "Nenhum marcador conhecido encontrado para relocalização.");
    }
}

void SLAMRelocalizationNode::send_pose_to_slam(const Sophus::SE3f &pose) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    Eigen::Quaternionf q = pose.unit_quaternion();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_msg.pose.position.x = pose.translation().x();
    pose_msg.pose.position.y = pose.translation().y();
    pose_msg.pose.position.z = pose.translation().z();

    pose_publisher_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Pose publicada: [%.3f, %.3f, %.3f]",
                pose.translation().x(), pose.translation().y(), pose.translation().z());
}
