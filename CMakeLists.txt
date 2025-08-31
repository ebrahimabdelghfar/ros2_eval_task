cmake_minimum_required(VERSION 3.8)
project(ros2_eval_task)

# ---------------- Dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# ---------------- Install Python Modules ----------------
ament_python_install_package(${PROJECT_NAME})
# ---------------- add libraries -------------------------
add_library(gazebo_utils_client SHARED
  ros2_eval_task/gazebo_utils_client.cpp
)
# ---------------- Set the include directories ------------
target_include_directories(gazebo_utils_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ---------------- Link dependencies to the library -------
ament_target_dependencies(gazebo_utils_client
  rclcpp
  gazebo_msgs
  geometry_msgs
  ament_index_cpp
  sensor_msgs
  cv_bridge
)
# ----------------- make the main file excutable ----------
add_executable(model_spawner_node ros2_eval_task/model_spawner_node.cpp)
# ----------------- Link dependencies to the executable -------
ament_target_dependencies(model_spawner_node
  rclcpp
  geometry_msgs
  gazebo_msgs
  ament_index_cpp
  sensor_msgs
  cv_bridge
)
# ----------------- Link the library to the executable -------
target_link_libraries(model_spawner_node 
  gazebo_utils_client
)
# ---------------- Install the executable -----------------
install(TARGETS
  gazebo_utils_client
  model_spawner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# ---------------- Install the header files ----------------
install(DIRECTORY include/
  DESTINATION include
)
# ---------------- Install Launch, RViz, and Worlds ----------------
install(DIRECTORY
  launch
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models/
  DESTINATION $ENV{HOME}/.gazebo/models
)

# ---------------- Install Package.xml ----------------
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
# Export the library for other packages to find and use
ament_export_include_directories(include)
ament_export_libraries(gazebo_utils_client)
# ---------------- Export ----------------
ament_package()

