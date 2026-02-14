import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 加载你的URDF模型
    pkg_myself_dog = get_package_share_directory('myself_dog')
    urdf_path = os.path.join(pkg_myself_dog, 'urdf', 'dog.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 2. 强制设置Harmonic版本的模型路径，彻底解决找不到STL的问题
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_myself_dog, '..') + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # 3. 官方标准启动Gazebo，适配Harmonic，绝对不会报「未找到命令」
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # 自动启动空世界，不用手动点RUN，去掉这行就会弹你熟悉的quick start窗口
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 4. 先启动URDF发布节点，解决启动顺序问题
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 5. 后启动关节状态发布，解决TF报错，带GUI滑块控制
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        arguments=[urdf_path]
    )

    # 6. 最后把机械狗加载到Gazebo里
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_dog',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'dog',
            '-z', '0.5'
        ]
    )

    return LaunchDescription([
        set_resource_path,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_robot
    ])