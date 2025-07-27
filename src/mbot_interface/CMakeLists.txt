cmake_minimum_required(VERSION 3.8)
project(mbot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(msg_files
  "msg/Person.msg"
  "msg/GridMap.msg"
  "msg/Pose2D.msg"
)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # 如果依赖其他消息包（如 std_msgs），需要添加
  DEPENDENCIES std_msgs
)


add_executable(interface_node src/main.cpp)

target_include_directories(interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(interface_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  interface_node
  "rclcpp"
  "std_msgs"
  "gazebo_ros"
  "gazebo_plugins"
  "urdf"
  "xacro"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Person.msg"
# )
rosidl_target_interfaces(interface_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(interface_node "${PROJECT_NAME}__rosidl_typesupport_cpp")


install(
  DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
)

install(
  DIRECTORY include/  # 要安装的目录（注意末尾的斜杠，只安装目录内的内容）
  DESTINATION include  # 安装到 install/include 目录
)

install(TARGETS interface_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
