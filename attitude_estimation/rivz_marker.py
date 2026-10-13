#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu

class OrientationMarkerNode(Node):
    def __init__(self):
        super().__init__('orientation_marker_node')

        # 初始化 Marker 物件
        self.marker = Marker()
        self.setup_marker()

        # 建立 Marker 發布器（ROS 2 的 queue_size 改為 qos_profile 的歷史記錄深度）
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # 訂閱 IMU 資料
        self.subscription = self.create_subscription(
            Imu,
            '/ekf/attitude',
            self.imu_callback,
            10
        )

    def setup_marker(self):
        self.marker.header.frame_id = "base_link"  # 根據需要調整 frame_id
        self.marker.ns = "orientation_marker"
        self.marker.id = 0
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD

        # 初始位置
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # 設定初始大小
        self.marker.scale.x = 20.0  # 長度
        self.marker.scale.y = 20.0  # 寬度
        self.marker.scale.z = 3.0   # 高度

        # 設定顏色
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0  # 不透明

    def imu_callback(self, data):
        # 從 IMU 訊息中獲取四元數姿態
        self.marker.pose.orientation = data.orientation
        
        # ROS 2 取得當前時間戳記的寫法
        self.marker.header.stamp = self.get_clock().now().to_msg()
        
        # 發布 Marker
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    
    node = OrientationMarkerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
