#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
from whole_body_control.msg import JointVelocity
import numpy as np

class WholeBodyController:
    def __init__(self):
        rospy.init_node('wbc_node', anonymous=True)
        # 订阅关节状态并发布wbc命令
        self.state_sub = rospy.Subscriber("/joint_states", JointState, self.state_callback)
        self.vel_pub = rospy.Publisher('/wbc_joint_velocities', JointVelocity, queue_size=10)
        self.rate = rospy.Rate(200)  

        self.joint_angles = [0.0, 0.0, 0.0]  
        self.joint_vels = [0.0, 0.0, 0.0]    
        self.y_h = 0.0                      
        # 无WBC控制的kd参数
        self.k_p = 50                     
        self.k_d = 0.6                      
        self.JointVel = [0.0, 0.0, 0.0]      

        
    def state_callback(self, msg):
        
        if len(msg.position) >= 4 and len(msg.velocity) >= 4:
            self.y_h = msg.position[0]                
            self.joint_angles[0] = msg.position[1]    
            self.joint_angles[1] = msg.position[2]   
            self.joint_angles[2] = msg.position[3]    

            self.joint_vels[0] = msg.velocity[1]       
            self.joint_vels[1] = msg.velocity[2]       
            self.joint_vels[2] = msg.velocity[3]      
        else:
            rospy.logwarn(f"joint_states数据不全：位置长度{len(msg.position)}，角速度长度{len(msg.velocity)}")

    def wbc_calculate(self):
        pass
    def no_wbc_calculate(self):
        """PD控制器+雅可比伪逆求解"""
        theta1 = self.joint_angles[0]
        theta2 = self.joint_angles[1]
        theta3 = self.joint_angles[2]
        
        theta12 = theta1 + theta2
        theta123 = theta1 + theta2 + theta3
        
        s1 = math.sin(theta1)
        s12 = math.sin(theta12)
        s123 = math.sin(theta123)
        
        c1 = math.cos(theta1)
        c12 = math.cos(theta12)
        c123 = math.cos(theta123)
        # 计算雅可比矩阵
        J = np.array([
            [-s1 - s12 - s123, -s12 - s123, -s123],
            [c1 + c12 + c123, c12 + c123, c123],
            [1.0, 1.0, 1.0]
        ])

        # 计算伪逆（求逆的话当机械臂运动超过工作空间会使整个系统崩溃）
        J_pinv = np.linalg.pinv(J)

        # 结合PD控制器计算期待的任务空间的速度
        x_d = np.array([2.0, -self.y_h, 0.0]).reshape(-1, 1)  
        x = np.array([
            c1 + c12 + c123, 
            s1 + s12 + s123, 
            theta123
        ]).reshape(-1, 1)  

        q_vel = np.array(self.joint_vels).reshape(-1, 1)  
        x_vel = J @ q_vel  

        x_dot = self.k_p * (x_d - x) - self.k_d * x_vel

        max_vel = 0.8  
        theta_dot = J_pinv @ x_dot
        theta_dot[0, 0] = np.clip(theta_dot[0, 0], -max_vel, max_vel)
        theta_dot[1, 0] = np.clip(theta_dot[1, 0], -max_vel, max_vel)
        theta_dot[2, 0] = np.clip(theta_dot[2, 0], -max_vel, max_vel)

        self.JointVel[0] = theta_dot[0, 0] 
        self.JointVel[1] = theta_dot[1, 0]  
        self.JointVel[2] = theta_dot[2, 0]  

        # 打印PD控制器关键参数（调试用）
        rospy.loginfo_throttle(1.0, 
            f"位置误差x：{x_d[0,0]-x[0,0]:.2f} | 末端速度x：{x_vel[0,0]:.2f} | "
            f"Kp={self.k_p} | Kd={self.k_d}"
        )

    def pub_joint_cmd(self):
        """发布关节角速度指令（不变）"""
        vel_msg = JointVelocity()
        vel_msg.joint_names = ["joint_1", "joint_2", "joint_3"]
        vel_msg.angle_vel = self.JointVel
        
        self.vel_pub.publish(vel_msg)
        
        rospy.loginfo_throttle(1.0, 
            f"joint1角速度：{self.JointVel[0]:.2f} | "
            f"joint2角速度：{self.JointVel[1]:.2f} | "
            f"joint3角速度：{self.JointVel[2]:.2f}"
        )

    def run(self):
        rospy.loginfo(f"WBC节点启动（PD控制器 Kp={self.k_p}, Kd={self.k_d}）...")
        rospy.sleep(0.5)
        
        while not rospy.is_shutdown():
            # self.no_wbc_calculate()
            self.wbc_calculate()
            self.pub_joint_cmd()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        wbc = WholeBodyController()
        wbc.run()
    except Exception as e:
        rospy.logerr(f"WBC节点运行出错：{e}")