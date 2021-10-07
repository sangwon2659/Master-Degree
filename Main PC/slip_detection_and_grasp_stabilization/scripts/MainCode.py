#!/usr/bin/env python

import rospy
import serial
import struct
import numpy as np
import threading
from keras.models import load_model
from slip_detection_and_grasp_stabilization.msg import FSS_sum_msg, Slip_Vector_msg, Slip_Detection_msg

# Number of lookbacks for the covariance computation
n_covariance = 10
# Number of sample frequencies required
n_frequency = 40
# Number of lookbacks of whole sequence of data
n_lookback = 0
# Scaling the covariance in order to match significance with the frequency data
scale_covariance = 1000
# Multiple for the normalization process
normalizer_constant = 8000000

# Using 2 of 5-Channel FSS Sensors
n_ch = 5

# Declaring an array to store data and display it later
# Variables for FSS and FSS sum
FSS = np.zeros(10)
FSS_previous = FSS
FSS_sum_temp = 0.0
FSS_sum = np.array(0.0)

# Variables for the Cov and FFT combined data and the classification result
Slip_Vector = np.zeros(n_frequency+1)
# Reference for deleting purposes of the lookback queue
reference = range((n_lookback+1)*(n_frequency+1), (n_lookback+1)*(n_frequency+1)+(n_frequency+1))
Slip_Vector_sum = Slip_Vector
# Binary classification result
Slip_NoSlip = 0.0

# Declaring variables for Covariance
Diff_Matrix = np.zeros((n_covariance, n_covariance))

# Declaring the USB port for which the arduino is connected
ser = serial.Serial(port = '/dev/ttyUSB0', baudrate=115200)
ser_ = serial.Serial(port = '/dev/ttyUSB1', baudrate=115200)

# Thread adjusted to 80Hz from the specifications of the FSS sensor
thread_ = 0.0125

# Loading model
MLP_Model = load_model('MLP_Model.h5')

def thread_run():
	global pub, pub2, pub3, Slip_NoSlip, Slip_Vector_sum, FSS_sum_temp, FSS

	# Publishing desired data
	#pub.publish(Slip_NoSlip)
	pub2.publish(Slip_Vector_sum)
	pub3.publish(FSS_sum_temp)
	#pub3.publish(FSS)

	# Displaying the data on the log
	threading.Timer(thread_, thread_run).start()

def talker():
	global pub, pub2, pub3, Slip_NoSlip, Slip_Vector_sum, FSS_sum_temp, FSS, FSS_previous, FSS_sum, Slip_Vector, Slip_Vector, Slip_NoSlip, Diff_Matrix, FSS
	
	# Initializing the node with the name 'SlipDetection'
	rospy.init_node('SlipDetection', anonymous=True)

	# Declaring publisher with respective topic names and message files
	pub = rospy.Publisher('Slip_NoSlip', Slip_Detection_msg, queue_size=1)
	pub2 = rospy.Publisher('Slip_Vector_sum', Slip_Vector_msg, queue_size=1)
	pub3 = rospy.Publisher('FSS_sum', FSS_sum_msg, queue_size= 1)
	#pub3 = rospy.Publisher('FSS', Slip_Vector_msg, queue_size=1)

	ser.reset_input_buffer()
	ser_.reset_input_buffer()
	thread_run()

	while not rospy.is_shutdown():

		# Reading the data from the arduino
		response = ser.readline()
		response_ = ser_.readline()

		# 5 bytes of data for each channel from the arduino + 1 byte of spacing
		# Needs more number of bytes compared to the ADC one
		if response.__len__() == n_ch*4+1 and response_.__len__() == n_ch*4+1:
			# A method of unpacking the data
			# There are 5 ls since the number of channels is 5
			# '<' for starting and l for n_ch bytes of long type data
			# Only considering upto the second last byte (response[:-1])
			# Inserting data from the 2 arduino into FSS array
			FSS[0:5] = np.array(struct.unpack('<lllll', response[:-1]))
			FSS[5:10] = np.array(struct.unpack('<lllll', response_[:-1]))
			
			# Normalization with normalizer_constant (8000000 = an approx of 2^23 is default)
			FSS = FSS/normalizer_constant
			
			# Summing up the 10-channel FSS data
			FSS_sum_temp = FSS.sum()
			
			# Filtering the sum data
			# If new sum not within boundary, it is set to be the previous sum
			if FSS_sum_temp > 1.5 or FSS_sum_temp < -0.1 and np.size(FSS_sum)>1:
				print(FSS_sum_temp)
				FSS_sum_temp = FSS_sum[-1]

			# Putting FSS data into FSS_sum queue
			FSS_sum = np.append(FSS_sum, FSS_sum_temp)

			if len(FSS_sum) > n_frequency*2:
				# Deleting earliest one in queue
				FSS_sum = np.delete(FSS_sum, [0])
				
				# Computing Covariance
				# Computing the difference between the latest and second lastest
				FSS_diff = FSS-FSS_previous
				# Inserting the data into Difference Matrix queue
				Diff_Matrix = np.vstack((FSS_diff, Diff_Matrix))
				# Deleting the earliest FSS_diff
				Diff_Matrix = np.delete(Diff_Matrix, n_covariance, axis=0)
				# Updating FSS_previous with the latest FSS data
				FSS_previous[:] = FSS[:]
				# Computing the covariance
				Cov_Matrix = np.dot(np.transpose(Diff_Matrix), Diff_Matrix)
				# Inserting the covariance value into the last position of Slip Vector array
				Slip_Vector[n_frequency] = np.sum(Cov_Matrix)-np.trace(Cov_Matrix)*scale_covariance

				# Computing FFT
				FFT_Vector = np.fft.fft(FSS_sum)
				# Filtering out the first data of FFT
				FFT_Vector = FFT_Vector[0:n_frequency]
				# Inserting the FFT data in to the first n_frequency positions of Slip Vector array
				Slip_Vector[0:n_frequency] = abs(FFT_Vector)

				# Inserting the Slip Vector array in to the Slip Vector sum queue
				Slip_Vector_sum = np.hstack((Slip_Vector, Slip_Vector_sum))
			
				# Compiling Slip Vectors and conducting the classification algorithm
				if len(Slip_Vector_sum) == (n_lookback+1)*(n_frequency+1) + (n_frequency+1):
					# Deleting the earliest Slip Vector with the reference array
					Slip_Vector_sum = np.delete(Slip_Vector_sum, reference)
					# Reshaping the Slip Vector sum to fit the classification algorithm
					Slip_Vector_sum_Reshaped = Slip_Vector_sum.reshape(1, (n_lookback+1)*(n_frequency+1))
					# Conducting the Classification algorithm for a binary output
					#Slip_NoSlip = MLP_Model.predict(Slip_Vector_sum_Reshaped)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
