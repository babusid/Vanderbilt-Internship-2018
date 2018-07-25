

#-------------------------------------------------------------------------------------#
#
#
#
# Created by Jacob Gloudemans and James Zhu
# 3/19/2018
#
#-------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------#

from interfaces import primary_interface, camera_interface, mic_interface
from mic_test import mic_test
import numpy
from array import array

#######################################################################################
# Value of this flag is True if robot detects a cliff immediately in front of it
#######################################################################################
class flag_cliff_detected:

	# Relevant constants
	CUTOFF = 3   

	def __init__(self, robot_name):

		# Variables for updating flag
		self.primary_int = primary_interface(robot_name)
		self.primary_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = False


	def update(self):

		# if either cliff sensor reads below the cutoff, set value to TRUE
		self.value = (self.primary_int.cliff[0] < flag_cliff_detected.CUTOFF) or (self.primary_int.cliff[1] < flag_cliff_detected.CUTOFF)

	def __repr__(self):		
		return "Cliff Detected:\t" + str(self.value)


#######################################################################################
# Value of this flag is True if robot detects a an object immediately in front of it
#######################################################################################
class flag_obj_detected:

	# Relevant constants
	CUTOFF = 0.15               # Distance at which an object triggers this flag  

	def __init__(self, robot_name):

		# Variables for updating flag
		self.primary_int = primary_interface(robot_name)
		self.primary_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = False


	def update(self):

		# if sonar sensor reads below cutoff and isn't zeroed out, value = True
		self.value = self.primary_int.sonar_range < flag_obj_detected.CUTOFF and self.primary_int.sonar_range != 0.0

	def __repr__(self):		
		return "Object Detected:\t" + str(self.value)

#######################################################################################
# Value of this flag is True if robot detects that it is being petted on its body
#######################################################################################
class flag_petting_detected: 

	def __init__(self, robot_name):

		# Variables for updating flag
		self.primary_int = primary_interface(robot_name)
		self.primary_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = False


	def update(self):

		# if any of the body touch sensors reads TRUE, set value to TRUE
		self.value = numpy.sum(array("B", self.primary_int.touch_body)) > 0

	def __repr__(self):
		return "Petting Detected:\t" + str(self.value)


#######################################################################################
# Value of this flag is True if robot detects a cliff immediately in front of it
#######################################################################################
class flag_tennis_ball_detected: 

	def __init__(self, robot_name):

		# Variables for updating flag
		self.camera_int = camera_interface(robot_name)
		self.camera_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = False


	def update(self):

		# THIS NEEDS TO BE COMPLLETED #

		# if either cliff sensor reads below the cutoff, set value to TRUE
		pass

	def __repr__(self):
		return "Tennis Ball Detected:\t" + str(self.value)



#######################################################################################
# Value of this flag is True if robot is in a dark environment, False otherwise
#######################################################################################
class flag_low_light: 

	# Relevant constants
	LIGHT_THRESHOLD = 25

	def __init__(self, robot_name):

		# Variables for updating flag
		self.primary_int = primary_interface(robot_name)
		self.primary_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = False

	def update(self):
		self.value = sum(self.primary_int.light) / 4 < flag_low_light.LIGHT_THRESHOLD

	def __repr__(self):
		return "Low light:\t" + str(self.value)

#######################################################################################
# Value of this flag is set to index of maximum cross correlation between left and right mics
#######################################################################################
class flag_audio: 

	def __init__(self, robot_name):

		# Variables for updating flag
		self.mic_int = mic_interface(robot_name)
		self.mic_int.relevant_flags.append(self)

		# Variables which store value of flag
		self.value = 0

	def update(self):
		if self.mic_int.left_mic:
			corr = numpy.ndarray.tolist(numpy.correlate(self.mic_int.left_mic,self.mic_int.right_mic,"same"))
			max_corr = max(corr)
			max_index = corr.index(max_corr)

			if max_corr > 50000:
				self.value = corr.index(max_corr)
			else: 
				self.value = 1
	def __repr__(self):
		return "Noise:\t" + str(self.value)

