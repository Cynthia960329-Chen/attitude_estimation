#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu

def imu_callback(data):
    global marker_pub, marker
    # 从IMU消息中获取四元数姿态
    marker.pose.orientation = data.orientation
    marker.header.stamp = rospy.Time.now()
    marker_pub.publish(marker)

def setup_marker():
    global marker
    marker = Marker()
    marker.header.frame_id = "base_link"  # 根据需要调整frame_id
    marker.ns = "orientation_marker"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 初始位置
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    # 设置初始大小
    marker.scale.x = 20  # 长度
    marker.scale.y = 20  # 宽度
    marker.scale.z = 3  # 高度

    # 设置颜色
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # 不透明

def main():
    global marker_pub
    rospy.init_node('orientation_marker_node')

    # 初始化Marker
    setup_marker()

    # 创建Marker发布器
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # 订阅IMU数据
    rospy.Subscriber('/ekf/attitude', Imu, imu_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
