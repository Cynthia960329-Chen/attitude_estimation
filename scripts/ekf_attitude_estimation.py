#!/usr/bin/env python3
import rospy
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
        return n_Dimension_object(4,self.x*other.x,self,y*other.y,self.z*other.z,self.w*other.w)
      else:
        return n_Dimension_object(3,self.x*other.x,self.y*other.y,self.z*other.z)
    else:
      if hasattr(self, 'w'):
        return n_Dimension_object(4,self.x*other,self.y*other,self.z*other,self.w*other)
      return n_Dimension_object(3,self.x*other,self.y*other,self.z*other)

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w  # 实数部分
        self.x = x  # 虚数部分 x
        self.y = y  # 虚数部分 y
        self.z = z  # 虚数部分 z

    @property
    def real(self):
        """返回实数部分"""
        return self.w

    @property
    def vector(self):
        """返回虚数向量部分"""
        return np.array([self.x, self.y, self.z])

    def norm(self):
        """计算四元数的范数"""
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self):
        """归一化四元数"""
        n = self.norm()
        if n == 0:
            raise ValueError("无法归一化零四元数")
        return Quaternion(self.w / n, self.x / n, self.y / n, self.z / n)

    def __mul__(self, other):
        """实现四元数乘法"""
        if not isinstance(other, Quaternion):
            raise TypeError("只能与另一个Quaternion相乘")
        
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(w, x, y, z)

    def conjugate(self):
        """返回四元数的共轭"""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def rotate(self, vector):
        """使用四元数旋转一个3D向量"""
        v = Quaternion(0, *vector)  # 将向量转换为四元数
        q_normalized = self.normalized()  # 确保四元数为单位四元数
        rotated_v = q_normalized * v * q_normalized.conjugate()  # 旋转公式
        return rotated_v.vector  # 只返回旋转后的向量部分
    def inv_rotate(self, vector):
        """使用四元数旋转一个3D向量"""
        v = Quaternion(0, *vector)  # 将向量转换为四元数
        q_normalized = self.normalized()  # 确保四元数为单位四元数
        rotated_v = q_normalized.conjugate() * v * q_normalized  # 旋转公式
        return rotated_v.vector  # 只返回旋转后的向量部分

def skew_symmetric_matrix(x):
  return np.array([[0, -x[2], x[1]],
                   [x[2], 0, -x[0]],
                   [-x[1], x[0], 0]])

DT = 1.0 / 10.0  # 假設IMU頻率為10Hz
G = 9.81  # 重力加速度
GNED = np.array([0.0, 0.0, -1.0])  # 重力加速度向量
# 磁場參考向量 (歸一化)
# 35度的磁傾角，-4度的磁偏角
inclination = 35 * np.pi / 180  # 轉換成弧度
declination = -4 * np.pi / 180  # 轉換成弧度

# 計算參考磁場向量（歸一化）
Bx = np.cos(inclination) * np.cos(declination)
By = np.cos(inclination) * np.sin(declination)
Bz = np.sin(inclination)

RNED = np.array([Bx, By, Bz])

COVW = 0.09  # 角速度噪聲


class AttitudeEstimationEKF:
  def __init__(self):
    self.init_ekf()
    self.set_ros()

  def init_ekf(self):
    # 修改為只追蹤四元數（去除bias）
    self.ekf = EKF(dim_x=4, dim_z=3)  # 狀態: 四元數, 測量: 加速度
    self.ekf.x = np.array([0.0, 0.0, 0.0, 1.0])  # [qx, qy, qz, qw]
    self.ekf.P *= 0.01
    self.ekf.Q = np.eye(4)
    self.ekf.R = np.diag(np.array([0.25, 0.25, 0.25]))  # 只有加速度的測量噪聲

  def set_ros(self):
    # 只訂閱 IMU 數據
    imu_sub = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
    self.pub = rospy.Publisher('/ekf/attitude', Imu, queue_size=10)

  def imu_callback(self, imu_msg):
    # 提取並轉換IMU數據
    acc = np.array([-imu_msg.linear_acceleration.x,
                    -imu_msg.linear_acceleration.y,
                    -imu_msg.linear_acceleration.z])
    
    gyro = n_Dimension_object(3,
                             imu_msg.angular_velocity.x,
                             imu_msg.angular_velocity.y,
                             imu_msg.angular_velocity.z)

    # 更新EKF
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
    # 更新狀態轉移矩陣（只考慮四元數部分）
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
    # 簡化的過程噪聲矩陣
    dt = DT
    sigma_w = 0.01
    self.ekf.Q = sigma_w * sigma_w * dt * np.eye(4)

  def HJacobian(self, x):
    # 計算測量矩陣（只考慮重力方向）
    q = Quaternion(w=x[3], x=x[0], y=x[1], z=x[2])
    g = GNED  # 假設GNED是定義好的重力向量 [0, 0, 1]
    
    H = np.array([
        [ g[0]*q.w + g[1]*q.z - g[2]*q.y, g[0]*q.x + g[1]*q.y + g[2]*q.z, -g[0]*q.y + g[1]*q.x - g[2]*q.w, -g[0]*q.z + g[1]*q.w + g[2]*q.x],
        [-g[0]*q.z + g[1]*q.w + g[2]*q.x, g[0]*q.y - g[1]*q.x + g[2]*q.w,  g[0]*q.x + g[1]*q.y + g[2]*q.z, -g[0]*q.w - g[1]*q.z + g[2]*q.y],
        [ g[0]*q.y - g[1]*q.x + g[2]*q.w, g[0]*q.z - g[1]*q.w - g[2]*q.x,  g[0]*q.w + g[1]*q.z - g[2]*q.y,  g[0]*q.x + g[1]*q.y + g[2]*q.z]
    ])
    
    return H

  def Hx(self, x):
    # 計算預測測量（只考慮重力方向）
    q = Quaternion(w=x[3], x=x[0], y=x[1], z=x[2])
    return q.inv_rotate(GNED)

  def publish_attitude(self, quat, acc, gyro):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
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

if __name__ == '__main__':
  rospy.init_node('ekf_attitude_estimation', anonymous=True)
  attitude_estimation = AttitudeEstimationEKF()
  rospy.spin()
