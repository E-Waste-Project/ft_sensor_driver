#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench
from std_msgs.msg import Float64


def res_cb(msg):
    # get the forces
    fx = msg.wrench.force.x
    fy = msg.wrench.force.y
    fz = msg.wrench.force.z
    
    # calculate the resultants
    res_xy = np.sqrt(fx**2 + fy**2)
    res_yz = np.sqrt(fy**2 + fz**2)
    res_xz = np.sqrt(fx**2 + fz**2)
    
    # resultant messages
    res_xy_msg = Float64(data=res_xy)
    res_yz_msg = Float64(data=res_yz)
    res_xz_msg = Float64(data=res_xz)

    # publish resultant messages
    res_xy_pub.publish(res_xy_msg)
    res_yz_pub.publish(res_yz_msg)
    res_xz_pub.publish(res_xz_msg)


rospy.init_node("res_pub")
rospy.Subscriber("ft_sensor_wrench/wrench/filtered", WrenchStamped, res_cb)
res_xy_pub = rospy.Publisher("ft_sensor_wrench/resultant/filtered/xy", Float64, queue_size=1)
res_yz_pub = rospy.Publisher("ft_sensor_wrench/resultant/filtered/yz", Float64, queue_size=1)
res_xz_pub = rospy.Publisher("ft_sensor_wrench/resultant/filtered/xz", Float64, queue_size=1)
rospy.spin()
