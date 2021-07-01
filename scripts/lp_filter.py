#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from scipy.signal import butter, sosfilt, sosfilt_zi
from collections import deque
from ros_numpy import numpify
import matplotlib.pyplot as plt


sampling_rate = 400
cutoff_freq = 5
num_samples = 100
zi =None

def raw_wrench_sub(wrench_msg):
    global zi, cutoff_freq, sampling_rate
    filtered_z, zi = butter_lowpass_filter([wrench_msg.wrench.force.z], cutoff_freq, sampling_rate, zi)
    msg = Float64()
    msg.data = filtered_z[0]
    z_filtered_pub.publish(msg)
    

def butter_lowpass_filter(data, cutoff, fs, z, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    sos = butter(order, normal_cutoff, btype='low', analog=False, output='sos')
    if z is None:
        z = sosfilt_zi(sos)
    y, zf = sosfilt(sos, data, zi=z)
    return y, zf

rospy.init_node("ft_lpr")
rospy.Subscriber("ft_sensor_wrench/wrench/raw", WrenchStamped, raw_wrench_sub)
xy_filtered_res_pub = rospy.Publisher("ft_sensor_wrench/resultant/filtered", Float64, queue_size=1)
z_filtered_pub = rospy.Publisher("ft_sensor_wrench/filtered_z", Float64, queue_size=1)
rospy.spin()
