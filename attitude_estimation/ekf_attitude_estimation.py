#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import message_filters
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF

class n_Dimension_object(object):
  def __init__(self, n, x, y, z, w=None):
    if n == 3:
      self.x = x
      self.y = y
      self.z = z
    if n == 4:
      self.x = x
      self.y = y
      self.z = z
      self.w = w
  # define scaler multiplier
  def __mul__(self, other):
    if type(other) == n_Dimension_object:
      if hasattr(other, 'w'):
        return n_Dimension_object(4, self.x * other.x, self.y * other.y, self.z * other.z, self.w * other.w)
      else:
        return n_Dimension_object(3, self.x * other.x, self.y * other.y, self.z * other.z)
    else:
      if hasattr(self, 'w'):
        return n_Dimension_object(4, self.x * other, self.y * other, self.z * other, self.w * other)
      return n_Dimension_object(3, self.x * other, self.y * other, self.z * other)

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w  # 實數部分
        self.x = x  # 虛數部分 x
        self.y = y  # 虛數部分 y
        self.z = z  # 虛數部分 z

    @property
    def real(self):
        """返回實數部分"""
        return self.w

    @property
    def vector(self):
        """返回虛數向量部分"""
        return np.array([self.x, self.y, self.z])

    def norm(self):
        """計算四元數的範數"""
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self):
        """歸一化四元數"""
        n = self.norm()
        if n == 0:
            raise ValueError("無法歸一化零四元數")
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def __mul__(self, other):
        """實現四元數乘法"""
        if not isinstance(other, Quaternion):
            raise TypeError("只能與另一個Quaternion相乘")
         
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

    def conjugate(self):
        """返回四元數的共軛"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def rotate(self, vector):
        """使用四元數旋轉一個3D向量"""
        v = Quaternion(0, *vector)  # 將向量轉換為四元數
        q_normalized = self.normalized()  # 確保四元數為單位四元數
        rotated_v = q_normalized * v * q_normalized.conjugate()  # 旋轉公式
        return rotated_v.vector  # 只返回旋轉後的向量部分

    def inv_rotate(self, vector):
        """使用四元數反旋轉一個3D向量"""
        v = Quaternion(0, *vector)  # 將向量轉換為四元數
        q_normalized = self.normalized()  # 確保四元數為單位四元數
        rotated_v = q_normalized.conjugate() * v * q_normalized  # 旋轉公式
        return rotated_v.vector  # 只返回旋轉後的向量部分

def skew_symmetric_matrix(x):
  return np.array([[0, -x[2], x[1]],
                   [x[2], 0, -x[0]],
                   [-x[1], x[0], 0]])

DT = 1.0 / 10.0  # 假設 IMU 頻率為 10Hz
G = 9.81  # 重力加速度
GNED = np.array([0.0, 0.0, -1.0])  # 重力加速度向量

# 磁場參考向量 (歸一化)
inclination = 35 * np.pi / 180  # 轉換成弧度
declination = -4 * np.pi / 180  # 轉換成弧度

Bx = np.cos(inclination) * np.cos(declination)
By = np.cos(inclination) * np.sin(declination)
Bz = np.sin(inclination)

RNED = np.array([Bx, By, Bz])
COVW = 0.09  # 角速度噪聲


class AttitudeEstimationEKF(Node):
  def __init__(self):
    super().__init__('ekf_attitude_estimation')
    self.init_ekf()
    self.set_ros()

  def init_ekf(self):
    # 修改為只追蹤四元數（去除 bias）
    self.ekf = EKF(dim_x=4, dim_z=3)  # 狀態: 四元數, 測量: 加速度
    self.ekf.x = np.array([0.0, 0.0, 0.0, 1.0])  # [qx, qy, qz, qw]
    self.ekf.P *= 0.01
    self.ekf.Q = np.eye(4)
    self.ekf.R = np.diag(np.array([0.25, 0.25, 0.25]))  # 只有加速度的測量噪聲

  def set_ros(self):
    # 替換為 ROS 2 的訂閱與發布語法
    self.imu_sub = self.create_subscription(
        Imu, 
        '/imu/data_raw', 
        self.imu_callback, 
        10
    )
    self.pub = self.create_publisher(Imu, '/ekf/attitude', 10)

  def imu_callback(self, imu_msg):
    # 提取並轉換 IMU 數據
    acc = np.array([-imu_msg.linear_acceleration.x,
                    -imu_msg.linear_acceleration.y,
                    -imu_msg.linear_acceleration.z])
     
    gyro = n_Dimension_object(3,
                              imu_msg.angular_velocity.x,
                              imu_msg.angular_velocity.y,
                              imu_msg.angular_velocity.z)

    # 更新 EKF
    self.predict(gyro)
     
    # 正規化四元數
    q = Quaternion(w=self.ekf.x[3], x=self.ekf.x[0], y=self.ekf.x[1], z=self.ekf.x[2])
    q = q.normalized()
    self.ekf.x[3] = q.real
    self.ekf.x[:3] = q.vector
     
    # 正規化加速度
    acc_normalized = acc / np.linalg.norm(acc)
    self.ekf.update(acc_normalized, self.HJacobian, self.Hx)

    # 再次正規化
    q = Quaternion(w=self.ekf.x[3], x=self.ekf.x[0], y=self.ekf.x[1], z=self.ekf.x[2])
    q = q.normalized()
    self.ekf.x[3] = q.real
    self.ekf.x[:3] = q.vector

    # 發布結果
    self.publish_attitude(self.ekf.x, acc, gyro)

  def predict(self, gyro):
    self.update_F_matrix(gyro)
    self.update_Q_matrix()
    self.ekf.predict()

  def update_F_matrix(self, gyro):
    dt = DT
    wx, wy, wz = gyro.x, gyro.y, gyro.z
     
    omega_matrix = np.array([
        [ 0,  wz, -wy,  wx],
        [-wz,   0,  wx,  wy],
        [ wy, -wx,   0,  wz],
        [-wx, -wy, -wz,   0]
    ])
     
    self.ekf.F = np.eye(4) + 0.5 * dt * omega_matrix

  def update_Q_matrix(self):
    dt = DT
    sigma_w = 0.01
    self.ekf.Q = sigma_w * sigma_w * dt * np.eye(4)

  def HJacobian(self, x):
    q = Quaternion(w=x[3], x=x[0], y=x[1], z=x[2])
    g = GNED  
     
    H = np.array([
        [ g[0]*q.w + g[1]*q.z - g[2]*q.y, g[0]*q.x + g[1]*q.y + g[2]*q.z, -g[0]*q.y + g[1]*q.x - g[2]*q.w, -g[0]*q.z + g[1]*q.w + g[2]*q.x],
        [-g[0]*q.z + g[1]*q.w + g[2]*q.x, g[0]*q.y - g[1]*q.x + g[2]*q.w,  g[0]*q.x + g[1]*q.y + g[2]*q.z, -g[0]*q.w - g[1]*q.z + g[2]*q.y],
        [ g[0]*q.y - g[1]*q.x + g[2]*q.w, g[0]*q.z - g[1]*q.w - g[2]*q.x,  g[0]*q.w + g[1]*q.z - g[2]*q.y,  g[0]*q.x + g[1]*q.y + g[2]*q.z]
    ])
     
    H = np.array(H, dtype=float)
    return H

  def Hx(self, x):
    q = Quaternion(w=x[3], x=x[0], y=x[1], z=x[2])
    return q.inv_rotate(GNED)

  def publish_attitude(self, quat, acc, gyro):
    imu_msg = Imu()
    # ROS 2 取得當前時間戳記的正確寫法
    imu_msg.header.stamp = self.get_clock().now().to_msg()
    imu_msg.header.frame_id = 'imu_frame'
     
    # 四元數
    imu_msg.orientation.x = quat[0]
    imu_msg.orientation.y = quat[1]
    imu_msg.orientation.z = quat[2]
    imu_msg.orientation.w = quat[3]
     
    # 轉換回原始座標系
    imu_msg.angular_velocity.x = -gyro.z
    imu_msg.angular_velocity.y = gyro.y
    imu_msg.angular_velocity.z = gyro.x
     
    imu_msg.linear_acceleration.x = -acc[2]
    imu_msg.linear_acceleration.y = -acc[1]
    imu_msg.linear_acceleration.z = acc[0]
     
    self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeEstimationEKF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
