import os
import sys
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction,ExecuteProcess,IncludeLaunchDescription, RegisterEventHandler,DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessExit 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # use_rviz = '--rviz2' in sys.argv
    # use_slam = '--slam' in sys.argv
     # 1. 声明启动参数（动态开关功能）
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='是否启动SLAM节点'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动RViz'
    )
    pkg_slam = get_package_share_directory('slam_toolbox')
    # 获取包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mbot = get_package_share_directory('mbot_pkg')

    # 世界文件路径
    world_file = os.path.join(pkg_mbot, 'worlds', 'room.world')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_mbot, 'urdf', 'mbot_gazebo.urdf.xacro')
    # print(urdf_file)
    doc = xacro.process_file(urdf_file)
    # print(doc)
    robot_description = {'robot_description': doc.toxml()} 
    # print(robot_description)
    
    # 启动Gazebo服务器
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # 启动Gazebo客户端
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    

    # 加载URDF到参数服务器
    load_urdf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    
    # 在Gazebo中生成机器人
    spawn_entity = TimerAction(
        period=3.0,  # 延迟时间（秒）
        actions=[  # 延迟后执行的动作（这里是启动节点）
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_mbot',
                output='screen',
                arguments=['-entity', 'mbot', '-topic', 'robot_description'],
            )
    ])

    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     name='spawn_mbot',
    #     output='screen',
    #     arguments=['-entity', 'mbot', '-topic', 'robot_description'],
    #     delay=3.0
    # )
    # spawn_entity = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_mbot, 'launch', 'spawn_mbot.launch.py')
    #     )
    # )

    # spawn_after_gazebo = RegisterEventHandler(
    #     OnProcessStart(matcher=lambda event: event.action.name == 'gzserver', on_start=[load_urdf,spawn_entity])
    # )
    
    # 启动SLAM节点
    slam_node = Node(
        package='slam',
        executable='slam_node',
        name='slam_node',
        output='screen'
    )
    # 3. 启动SLAM Toolbox（实时建图，核心参数：仿真时间、建图模式）
    # slam_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
    #     launch_arguments={
    #         'use_sim_time': 'true',  # 匹配Gazebo仿真时间
    #         'params_file': os.path.join(pkg_mbot, 'config', 'slam_params.yaml'),  # 自定义SLAM参数
    #         'use_ros2_control': 'false'  # TB3仿真无需ros2_control
    #     }.items()
    # )
    # 启动control节点
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen'
    )

     #启动navi节点
    navigation_node = Node(
        package='navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    rviz2_node = Node(package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')))
    
    # gazebo_pose_publisher_node = Node(
    #     package='gazebo_pose_publisher',
    #     executable='gazebo_pose_publisher',
    #     name='gazebo_pose_publisher',
    #     output='screen',
    #     parameters=[{'robot_name': 'mbot', 'publish_topic': '/mbot/gazebo_pose', 'update_rate': 30.0, 'frame_id': 'world'}]
    # )   

    Nodes = [
        declare_use_slam,
        declare_use_rviz,
        gazebo_server,
        gazebo_client,
        load_urdf,
        spawn_entity,
        # gazebo_pose_publisher_node,
        slam_node, 
        navigation_node,
        # control_node,
        rviz2_node]
    # if use_slam:
    #     print("use slam")
    #     Nodes.append(slam_node)

    # return LaunchDescription([
    #     gazebo_server,
    #     gazebo_client,
    #     load_urdf,
    #     spawn_entity,
    #     slam_node, 
    #     control_node
    # ])
    return LaunchDescription(Nodes)