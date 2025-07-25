import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mbot = get_package_share_directory('mbot_pkg')

    # 世界文件路径
    world_file = os.path.join(pkg_mbot, 'worlds', 'empty_world.world')
    
    # URDF文件路径
    urdf_file = os.path.join(pkg_mbot, 'urdf', 'mbot_gazebo.urdf.xacro')
    print(urdf_file)
    doc = xacro.process_file(urdf_file)
    print(doc)
    robot_description = {'robot_description': doc.toxml()} 
    
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
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_mbot',
        output='screen',
        arguments=['-entity', 'mbot', '-topic', 'robot_description']
    )
    
    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        load_urdf,
        spawn_entity
    ])