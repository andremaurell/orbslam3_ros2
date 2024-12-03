#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "common.hpp"

// Definição da variável global slam_data
SLAMData slam_data;

// Função para processar os marcadores ArUco
void process_aruco_tags(cv::Mat &frame, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
{
    // Ensure the frame is non-empty
    if (frame.empty()) {
        std::cerr << "Error: Frame is empty!" << std::endl;
        return;
    }

    // Ensure matrices are valid
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        throw std::runtime_error("Camera parameters are not loaded properly.");
    }

    // Ensure matrices are of correct type
    cv::Mat cam_mat, dist_coeffs_mat;
    camera_matrix.convertTo(cam_mat, CV_64F);
    dist_coeffs.convertTo(dist_coeffs_mat, CV_64F);

    // Configure the dictionary for ArUco markers
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    // Detect ArUco markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame, dictionary, corners, ids);

    if (!ids.empty()) {
        // Draw detected markers
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            int marker_id = ids[i];

            // Estimate marker pose
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cam_mat, dist_coeffs_mat, rvecs, tvecs);

            Eigen::Vector3f marker_translation(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            Eigen::AngleAxisf rotation(Eigen::AngleAxisf(rvecs[i][0], Eigen::Vector3f::UnitX()) *
                                        Eigen::AngleAxisf(rvecs[i][1], Eigen::Vector3f::UnitY()) *
                                        Eigen::AngleAxisf(rvecs[i][2], Eigen::Vector3f::UnitZ()));
            Sophus::SE3f normalized_pose(rotation.toRotationMatrix(), marker_translation);

            slam_data.marker_ids.push_back(marker_id);
            slam_data.marker_poses.push_back(normalized_pose);

            std::cout << "Detected ArUco Marker ID: " << marker_id
                      << " Position: (" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << ")" << std::endl;

            // Draw axis for visualization
            cv::aruco::drawAxis(frame, cam_mat, dist_coeffs_mat, rvecs[i], tvecs[i], 0.1);
        }
    }

    // Display updated frame
    cv::imshow("ArUco Detection", frame);
    cv::waitKey(1);
}
