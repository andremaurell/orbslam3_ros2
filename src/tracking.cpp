#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "common.hpp"

class SLAMRelocalizationNode : public rclcpp::Node {
public:
    SLAMRelocalizationNode()
        : Node("slam_relocalization_node") {
        // Subscrição ao tópico de imagem
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/freedom_vehicle/camera/image_raw", 10,
            std::bind(&SLAMRelocalizationNode::imageCallback, this, std::placeholders::_1));

        // Carregar marcadores conhecidos
        try {
            load_markers_from_json("/home/lognav/colcon_ws/src/orbslam3_ros2/config/arucos_infos.json", known_markers_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load markers: %s", e.what());
        }

        RCLCPP_INFO(this->get_logger(), "SLAM Relocalization Node initialized.");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Converter imagem ROS2 para OpenCV
        cv::Mat frame = convert_ros_image_to_cv(msg);

        // Detectar ArUcos com marcadores conhecidos
        process_aruco_tags(frame, camera_matrix_, dist_coeffs_, known_markers_);

        // Lógica de relocalização
        if (!slam_data.marker_ids.empty()) {
            for (size_t i = 0; i < slam_data.marker_ids.size(); ++i) {
                int id = slam_data.marker_ids[i];
                if (known_markers_.count(id)) {
                    Sophus::SE3f aruco_pose_in_map = known_markers_[id];
                    Sophus::SE3f camera_pose = aruco_pose_in_map * slam_data.marker_poses[i].inverse();

                    // Log da posição da câmera no mundo
                    Eigen::Vector3f camera_position = camera_pose.translation();
                    RCLCPP_INFO(this->get_logger(), "Relocalizado com ArUco ID: %d", id);
                    RCLCPP_INFO(this->get_logger(), "Posição da câmera no mundo: [%.3f, %.3f, %.3f]",
                                camera_position[0], camera_position[1], camera_position[2]);

                    // Enviar a pose para o ORB-SLAM3
                    send_pose_to_slam(camera_pose);
                    break;
                }
            }
        }
    }

    void send_pose_to_slam(const Sophus::SE3f &pose) {
        RCLCPP_INFO(this->get_logger(), "Sending pose to ORB-SLAM3...");
        // TODO: Implementar comunicação com o ORB-SLAM3
    }

    cv::Mat convert_ros_image_to_cv(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Converter sensor_msgs::msg::Image para cv::Mat usando cv_bridge
        try {
            return cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao converter imagem: %s", e.what());
            return cv::Mat();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::Mat camera_matrix_, dist_coeffs_;
    std::map<int, Sophus::SE3f> known_markers_;
};
