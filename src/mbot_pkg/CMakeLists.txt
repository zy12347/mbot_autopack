cmake_minimum_required(VERSION 3.8)
project(mbot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mbot_interface REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# message(WARNING "mbot_interface 头文件路径: ${mbot_interface_INCLUDE_DIRS}")
# message(WARNING "mbot_interface 库路径: ${mbot_interface_LIBRARIES}")
# message(WARNING "mbot_interface 包路径: ${mbot_interface_DIR}")

# find_package(gazebo_ros_diff_drive REQUIRED)

add_executable(mbot_manager src/mbot_manager.cpp)

target_compile_features(mbot_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mbot_manager
  "rclcpp"
  "std_msgs"
  "mbot_interface"
  "gazebo_ros"
  "gazebo_plugins"
  "urdf"
  "xacro"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)

target_include_directories(mbot_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  # get_target_property(MBOT_MANAGER_INCLUDES mbot_manager INCLUDE_DIRECTORIES)
  # message(WARNING "mbot_manager 实际头文件路径: ${MBOT_MANAGER_INCLUDES}")

install(TARGETS mbot_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.world"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
