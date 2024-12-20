cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# Define o PYTHONPATH para os pacotes do ROS
set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Define o padrão de C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontra os pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(aruco_ros REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenCV REQUIRED)  # Adicionado para OpenCV

# Inclui os diretórios necessários
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/json/include
  ${aruco_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # Diretórios do OpenCV
)

link_directories(
  include
)

# Define os executáveis e suas dependências
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
  src/tracking.cpp
  src/common.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV nav_msgs visualization_msgs rviz_visual_tools tf2_eigen)
target_link_libraries(mono
  ${ORB_SLAM3_LIBRARIES}
  ${aruco_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
  src/common.cpp 
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV nav_msgs visualization_msgs rviz_visual_tools tf2_eigen)
target_link_libraries(rgbd
  ${ORB_SLAM3_LIBRARIES}
  ${aruco_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
  src/common.cpp 
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin OpenCV nav_msgs visualization_msgs rviz_visual_tools tf2_eigen)
target_link_libraries(stereo
  ${ORB_SLAM3_LIBRARIES}
  ${aruco_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV nav_msgs visualization_msgs rviz_visual_tools tf2_eigen)
target_link_libraries(stereo-inertial
  ${ORB_SLAM3_LIBRARIES}
  ${aruco_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
)

# Instala os executáveis
install(TARGETS mono rgbd stereo stereo-inertial DESTINATION lib/${PROJECT_NAME})

# Inclui os diretórios para cada executável
target_include_directories(mono PRIVATE ${ORB_SLAM3_ROOT_DIR}/Thirdparty/json/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(rgbd PRIVATE ${ORB_SLAM3_ROOT_DIR}/Thirdparty/json/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(stereo PRIVATE ${ORB_SLAM3_ROOT_DIR}/Thirdparty/json/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(stereo-inertial PRIVATE ${ORB_SLAM3_ROOT_DIR}/Thirdparty/json/include ${OpenCV_INCLUDE_DIRS})

# Finaliza a configuração do ament
ament_package()
