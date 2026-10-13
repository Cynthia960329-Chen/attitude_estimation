import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 取得 attitude_estimation 套件在 share 目錄下的安裝路徑
    pkg_share = get_package_share_directory('attitude_estimation')
    
    # 對應原本的 rviz 設定檔路徑
    rviz_config_path = os.path.join(pkg_share, 'launch', 'rviz', 'rviz.rviz')

    # 1. 定義 arg 參數（對應原本的 <arg name="open_rviz" default="true"/>）
    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Whether to open RViz'
    )

    # 取得參數的值
    open_rviz = LaunchConfiguration('open_rviz')

    # 2. EKF 姿態估測節點
    ekf_node = Node(
        package='attitude_estimation',
        executable='ekf_attitude_estimation',
        name='ekf_attitude_estimation',
        output='screen'
    )

    # 3. RViz Marker 視覺化節點
    marker_node = Node(
        package='attitude_estimation',
        executable='rivz_marker',
        name='rivz_marker',
        output='screen'
    )

    # 4. RViz 節點（加上條件判斷，當 open_rviz 為 true 時才會啟動）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(open_rviz)
    )

    return LaunchDescription([
        open_rviz_arg,
        ekf_node,
        marker_node,
        rviz_node
    ])
