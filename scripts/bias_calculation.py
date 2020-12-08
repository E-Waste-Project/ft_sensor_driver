import nidaqmx
import numpy as np


with nidaqmx.Task() as task:
	task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai1")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai2")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai3")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai4")
	task.ai_channels.add_ai_voltage_chan("Dev1/ai5")
 
	data = task.read(number_of_samples_per_channel=1000000, timeout=1000)
	bias_samples = np.array(data)
	bias_mean = np.mean(bias_samples, axis=1)
	
	print bias_mean