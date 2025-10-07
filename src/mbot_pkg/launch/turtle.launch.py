import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 路径配置
    pkg_my = get_package_share_directory('mbot_pkg')  # 你的功能包
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')  # 包含机器人启动文件
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')  # Gazebo核心包
    pkg_slam = get_package_share_directory('slam_toolbox')

    # 自定义世界文件路径（你的room.world）
    custom_world_path = os.path.join(pkg_my, 'worlds', 'room.world')

    # 2. 配置参数（可通过命令行动态传入，如x_pose调整机器人初始位置）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')  # 机器人初始X坐标
    y_pose = LaunchConfiguration('y_pose', default='0.0')  # 机器人初始Y坐标

    # 3. 启动Gazebo服务器（加载自定义世界）
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': custom_world_path  # 关键：传入你的自定义世界路径
        }.items()
    )

    # 4. 启动Gazebo客户端（图形界面）
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 5. 启动机器人状态发布器（TF变换、关节状态等）
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. 在Gazebo中生成TurtleBot3机器人
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
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

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # arguments=['-d', os.path.join(pkg_my, 'rviz', 'slam_nav_rviz.rviz')]  # 自定义RViz配置（见下文）
    )
    # 7. 组装所有启动项
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)       # 启动Gazebo服务器（加载自定义世界）
    ld.add_action(gzclient_cmd)       # 启动Gazebo客户端
    ld.add_action(robot_state_publisher_cmd)  # 启动机器人状态发布器
    ld.add_action(spawn_turtlebot_cmd)        # 生成机器人
    ld.add_action(slam_launch)
    ld.add_action(rviz2_node)

    return ld
    