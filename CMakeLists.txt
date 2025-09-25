cmake_minimum_required(VERSION 3.8)
project(gocargo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- หา packages ที่ต้องใช้ ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)

# --- สร้าง executable ---
#add_executable(laser_filter_node src/laser_filter_node.cpp)

# --- ระบุ dependencies ของ target ---
#ament_target_dependencies(laser_filter_node
# rclcpp
#  sensor_msgs
#  std_msgs
#)

# --- ติดตั้งไฟล์ ---
install(TARGETS
  #laser_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY meshes urdf launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

# --- ตั้งค่า testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
