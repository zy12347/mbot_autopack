from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 路径配置（替换为你的功能包名）
    # pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_tb3_sim = get_package_share_directory('turtlebot3_gazebo')
    pkg_my = get_package_share_directory('mbot_pkg')  # 你的功能包
    pkg_slam = get_package_share_directory('slam_toolbox')

    custom_world_path = os.path.join(pkg_my, 'worlds', 'room.world')
    print(custom_world_path)
    # 2. 启动Gazebo（空世界+TB3机器人）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_tb3_sim, 'launch', 'empty_world.launch.py')),
        launch_arguments={'world': custom_world_path,'use_rviz': 'false'}.items()
    )

    # 3. 启动SLAM Toolbox（实时建图，核心参数：仿真时间、建图模式）
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',  # 匹配Gazebo仿真时间
            'params_file': os.path.join(pkg_my, 'config', 'slam_params.yaml'),  # 自定义SLAM参数
            'use_ros2_control': 'false'  # TB3仿真无需ros2_control
        }.items()
    )

    # # 4. 启动Nav2（含Hybrid A*规划器 + AMCL定位）
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
    #     launch_arguments={
    #         'params_file': os.path.join(pkg_my, 'config', 'nav2_hybrid_astar_amcl_params.yaml'),  # 含AMCL配置
    #         'use_sim_time': 'true',
    #         'map_topic': '/map'  # 订阅SLAM生成的实时地图
    #     }.items()
    # )

    # 5. 启动RViz2（加载包含“实时地图+机器人位姿”的配置）
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # arguments=['-d', os.path.join(pkg_my, 'rviz', 'slam_nav_rviz.rviz')]  # 自定义RViz配置（见下文）
    )

    # 6. （可选）Gazebo真值发布节点（将/model_states转为机器人单独位姿话题，方便查看）
    # gazebo_pose_node = Node(
    #     package='rviz2',  # 用简单节点转发，也可自定义Python脚本
    #     executable='topic_tools',
    #     name='gazebo_pose_extractor',
    #     output='screen',
    #     arguments=['relay', '/gazebo/model_states', '/robot_true_pose', '--filter', 'msg.name=="turtlebot3_waffle"']
    # )

    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        # nav2_launch,
        rviz2_node,
        # gazebo_pose_node  # 可选，按需添加
    ])

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # 获取包的路径
#     package_dir = get_package_share_directory('mbot_pkg')  # 替换为你的包名
#     # 地图YAML文件路径
#     map_yaml_path = os.path.join(package_dir, 'maps', 'map.yaml')

#     return LaunchDescription([
#         # 启动地图服务器节点
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'yaml_filename': map_yaml_path}]  # 指定YAML文件路径
#         ),
#         # 启动生命周期管理器（管理地图服务器状态）
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_map',
#             output='screen',
#             parameters=[
#                 {'node_names': ['map_server']},  # 管理的节点名
#                 {'autostart': True}              # 自动启动
#             ]
#         )
#     ])
