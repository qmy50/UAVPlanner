#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('cylinder_controller')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(50)               # 发布频率与updateRate一致
twist = Twist()

# 示例：使圆柱沿一条8字形轨迹运动
t = 0.0
while not rospy.is_shutdown():
    # 计算随时间变化的线速度和角速度
    # 解析得到 x 方向线速度做正弦变化，z 方向角速度也做正弦变化
    twist.linear.x = 0.5 * t
    twist.angular.z = 0.5 * t
    pub.publish(twist)
    rate.sleep()
    t += 0.02