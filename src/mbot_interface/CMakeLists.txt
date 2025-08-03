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
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2 REQUIRED)

  set(msg_files
    "msg/Person.msg"
    "msg/GridMap.msg"
    "msg/Pose2D.msg"
  )

  # 生成消息
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
  )

  # 创建静态库包含 LaserScan 和 Odom 的实现
  file(GLOB MBOT_INTERFACE_SRC
    "src/*.cpp"
  )
  add_library(mbot_interface_lib STATIC
    ${MBOT_INTERFACE_SRC}
  )

  # 确保消息在库之前生成
  add_dependencies(mbot_interface_lib
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )

  target_include_directories(mbot_interface_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

  target_compile_features(mbot_interface_lib PUBLIC c_std_99 cxx_std_17)

  ament_target_dependencies(mbot_interface_lib
    "rclcpp"
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "nav_msgs"
    "tf2"
    "rosidl_default_runtime"
  )

  rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
  
  # 链接消息类型支持库（关键部分）
  target_link_libraries(mbot_interface_lib "${cpp_typesupport_target}")

  # 安装头文件
  install(
    DIRECTORY include/
    DESTINATION include
  )

  # 安装消息文件
  install(
    DIRECTORY msg/
    DESTINATION share/${PROJECT_NAME}/msg
  )

  # 安装静态库并导出目标（核心）
  install(TARGETS mbot_interface_lib
    EXPORT mbot_interface_libTargets
    ARCHIVE DESTINATION lib  # 静态库安装路径
    INCLUDES DESTINATION include
  )

  # 导出配置
  ament_export_include_directories(include)
  ament_export_targets(mbot_interface_libTargets  HAS_LIBRARY_TARGET)
  ament_export_dependencies(rosidl_default_runtime)


  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
