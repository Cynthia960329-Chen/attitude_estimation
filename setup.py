from setuptools import setup
import os
from glob import glob

package_name = 'attitude_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 安裝 package.xml 標記
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安裝 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安裝整個 launch 資料夾（包含 rviz 設定檔）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'rviz'), glob('launch/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrlrobot',
    maintainer_email='mrlrobot@todo.todo',
    description='Attitude estimation EKF package for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_attitude_estimation = attitude_estimation.ekf_attitude_estimation:main',
            'rivz_marker = attitude_estimation.rivz_marker:main',
        ],
    },
)
