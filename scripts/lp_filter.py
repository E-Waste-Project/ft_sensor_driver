#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped, Wrench
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from scipy.signal import butter, filtfilt
from collections import deque
from ros_numpy import numpify
import matplotlib.pyplot as plt


sampling_rate = 400
cutoff_freq = 10
num_samples = 100
fx_queue = deque(maxlen=num_samples)
fy_queue = deque(maxlen=num_samples)
fz_queue = deque(maxlen=num_samples)
queue = deque(maxlen=num_samples)


def raw_wrench_sub(wrench_msg):
    global fx_queue, fy_queue, fz_queue
    # push the new message in queue
    fx_queue.append(wrench_msg.wrench.force.x)
    fy_queue.append(wrench_msg.wrench.force.y)
    fz_queue.append(wrench_msg.wrench.force.z)
    # convert to array
    fx_raw_arr = np.array(list(fx_queue))
    fy_raw_arr = np.array(list(fy_queue))
    fz_raw_arr = np.array(list(fz_queue))
    # filter the messages
    fx_filtered_arr = butter_lowpass_filter(fx_raw_arr, cutoff_freq, sampling_rate)
    fy_filtered_arr = butter_lowpass_filter(fy_raw_arr, cutoff_freq, sampling_rate)
    fz_filtered_arr = butter_lowpass_filter(fz_raw_arr, cutoff_freq, sampling_rate)
    # get xy resultant
    res = np.sqrt(fx_filtered_arr[-1]**2 + fy_filtered_arr[-1]**2)
    res_filtered_msg = Float64(data=res)
    # publish xy resultant
    xy_filtered_res_pub.publish(res_filtered_msg)
    # publish z
    z_filtered_pub.publish(fz_filtered_arr[-1]) 

def butter_lowpass_filter(data, cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data, axis=0, padlen=0)
    return y

rospy.init_node("ft_lpr")
rospy.Subscriber("ft_sensor_wrench/wrench/raw", WrenchStamped, raw_wrench_sub)
xy_filtered_res_pub = rospy.Publisher("ft_sensor_wrench/resultant/filtered", Float64, queue_size=1)
z_filtered_pub = rospy.Publisher("ft_sensor_wrench/filtered_z", Float64, queue_size=1)
rospy.spin()
