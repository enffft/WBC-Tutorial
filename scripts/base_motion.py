#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from whole_body_control.msg import JointVelocity
import math
import numpy as np
import threading

# 通用角度归一化函数
def normalize_angle(angle):
    normalized = angle % (2 * math.pi)
    if normalized > math.pi:
        normalized -= 2 * math.pi
    return normalized

class ArmCycleController:
    def __init__(self):
        rospy.init_node('arm_cycle_controller', anonymous=True)
        
        # 发布关节状态（位置+角速度）
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(200)  
        self.dt = self.rate.sleep_dur.to_sec() 
    
        self.joint_names = ["slide_joint", "joint_1", "joint_2", "joint_3"]
        
        self.slide_amplitude = 2.0    # 幅值
        self.slide_frequency = 0.025    # 频率
        self.current_slide_pos = 0.0  
        
        # 关节状态初始化
        self.joint_angles = [math.pi / 3, -2 * math.pi / 3, 0.0]  
        self.wbc_joint_vels = [0.0, 0.0, 0.0]  
        
        # 订阅WBC发布的角速度（加锁保证线程安全）
        self.wbc_vel_lock = threading.Lock()
        self.vel_sub = rospy.Subscriber(
            '/wbc_joint_velocities', 
            JointVelocity, 
            self.wbc_vel_callback
        )

    def wbc_vel_callback(self, msg):
        with self.wbc_vel_lock:
            expected_joint_names = ["joint_1", "joint_2", "joint_3"]
            if msg.joint_names != expected_joint_names or len(msg.angle_vel) != 3:
                rospy.logwarn(f"WBC消息格式错误，期望关节名{expected_joint_names}，角速度长度3")
                return
            self.wbc_joint_vels = list(msg.angle_vel)

    def update_slide_position(self):
        t = rospy.Time.now().to_sec()
        self.current_slide_pos = self.slide_amplitude * math.sin(2 * math.pi * self.slide_frequency * t)
        rospy.loginfo_throttle(1.0, f"基座当前位置：{self.current_slide_pos:.2f}m（幅值：{self.slide_amplitude}m）")

    def update_joint_angles(self):
        with self.wbc_vel_lock:
            joint1_vel = self.wbc_joint_vels[0]
            joint2_vel = self.wbc_joint_vels[1]
            joint3_vel = self.wbc_joint_vels[2]
        
        self.joint_angles[0] += joint1_vel * self.dt
        self.joint_angles[1] += joint2_vel * self.dt
        self.joint_angles[2] += joint3_vel * self.dt

        self.joint_angles = [normalize_angle(ang) for ang in self.joint_angles]

    def publish_joint_state(self):
        """发布更新后的关节位置+角速度"""
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = self.joint_names
        
        joint_msg.position = [self.current_slide_pos] + self.joint_angles
        
        with self.wbc_vel_lock:
            t = rospy.Time.now().to_sec()
            slide_vel = 2 * math.pi * self.slide_frequency * self.slide_amplitude * math.cos(2 * math.pi * self.slide_frequency * t)
            joint_msg.velocity = [slide_vel] + self.wbc_joint_vels
        
        self.joint_pub.publish(joint_msg)

    def run(self):
        rospy.loginfo(f"运动控制节点启动 → 基座幅值：{self.slide_amplitude}m，频率：{self.slide_frequency}Hz")
        rospy.sleep(0.5)  
        
        while not rospy.is_shutdown():
            self.update_slide_position()  # 更新基座位置
            self.update_joint_angles()    # 更新旋转关节角度（WBC控制）
            self.publish_joint_state()    # 发布关节状态
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ArmCycleController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("运动控制节点停止")
    