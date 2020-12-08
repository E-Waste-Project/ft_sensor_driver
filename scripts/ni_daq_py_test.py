#!/usr/bin/env python
import nidaqmx
import rospy
import numpy as np
from geometry_msgs.msg import Wrench

bias = np.array([-0.63452622, -0.1221085,   0.12652013,  0.21039776, -0.27837171,  0.45799553])

transformation_matrix = np.array([[-0.023653656180277, 0.013498529819632, 0.21932096679904, -3.49237283000002, -0.260976645567291, 3.76683279822371],
                                 [-0.27509758285346, 4.18175481585886, 0.06683548240138, -2.0143756008427, 0.161855269400006, -2.18500540342714],
                                 [6.54419998573492, -0.37515063215338, 6.69545217350523, -0.458698390024187, 6.64864170271365, -0.351508483900239], 
                                 [-0.003065032809969, 0.050837610370019, -0.191309874107432, -0.011682495430063, 0.195310317930836, -0.036035209146095],
                                 [0.217770789204777, -0.012313946101769, -0.114575684516563, 0.049325825236666, -0.106740349421393, -0.039094333983985],
                                 [0.008003163374078, -0.112344720021756, 0.00643395871361, -0.107936745804472, 0.006696875526182, -0.117835960308104]])



rospy.init_node("ft_sensor")
pub = rospy.Publisher("ft_topic", Wrench, queue_size=1)
rospy.sleep(1)



# print transformation_matrix[0, :]
with nidaqmx.Task() as task:
	task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai3")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai4")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai5")


	try:
		while not rospy.is_shutdown():
			data = task.read(number_of_samples_per_channel=1000)
   			# print data 
			raw_data = np.array(data)

			mean_data = np.mean(raw_data, axis=1)
			biased_data = mean_data - bias
			wrench = np.matmul(transformation_matrix, biased_data)
			wrench_msg = Wrench()
			wrench_msg.force.x = wrench[0]
			wrench_msg.force.y = wrench[1]
			wrench_msg.force.z = wrench[2]
			wrench_msg.torque.x = wrench[3]
			wrench_msg.torque.y = wrench[4]
			wrench_msg.torque.z = wrench[5]
			print wrench_msg
			pub.publish(wrench_msg)
	except rospy.ROSInterruptException:
		pass
		