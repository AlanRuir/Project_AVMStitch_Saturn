import os
import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('avmstitch'), 'config', 'avmstitch.yaml')

    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"avmstitch_{current_time}")
    print("log_dir: ", log_dir)

    os.makedirs(log_dir, exist_ok=True)  # 创建log目录

    declare_log_dir = DeclareLaunchArgument(        # 声明log目录
        'log_dir',
        default_value=log_dir,
        description='Directory for log files'
    )
    print("declare_log_dir: ", declare_log_dir)

    # 设置 ROS_LOG_DIR 环境变量
    set_log_dir_env = SetEnvironmentVariable(
        'ROS_LOG_DIR', LaunchConfiguration('log_dir')
    )

    avmstitch_node = Node(
        package='avmstitch',
        executable='avmstitch',
        name='avmstitch_node',
        output='log',
        parameters=[{'log_dir': LaunchConfiguration('log_dir')}, config_file]
    )

    return LaunchDescription([
        declare_log_dir,        # 声明log目录
        set_log_dir_env,        # 设置 ROS_LOG_DIR 环境变量
        avmstitch_node
    ])
