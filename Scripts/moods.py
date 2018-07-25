
#-------------------------------------------------------------------------------------#
#
#
#
# Created by Jacob Gloudemans and James Zhu
# 3/19/2018
#
#-------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------#


from interfaces import primary_interface, mic_interface, sound_interface
import random
import numpy
import time
from mic_test import mic_test


#######################################################################################
#  
# Relaxed mood is characterized by an average blink rate and blink speed with an average  
# amout of random movement. This is the default mood when the robot is initialized. The
#
#######################################################################################

class mood_relaxed:

	# Movement params
	DRIVE_FREQ = 8.0               # Avg time between movements
	AVG_DRIVE_SPEED = 0.2           # Avg linear drive speed
	DRIVE_SPEED_SIGMA = 0.05        # Std Dev of linear drive speed
	AVG_TURN_SPEED = 0.0            # Avg angular drive speed
	TURN_SPEED_SIGMA = 0.8          # Std Dev of angular drive speed
	AVG_LIN_DIST = 1.0              # Avg meters travelled during a movement

	#Head params
	HEAD_POSITION = [0,.5,0,0]
	HEAD_SPEED = [1,1,1,1]

	# Eyelid params
	BLINK_SPEED = 0.25
	AVG_BLINK_RATE = 10.0           # Avg time between blinks
	AVG_EYELID_DROOP = 0.4

	# Tail param
	TAIL_WAG = 0.0

	# LED params
	LED_COLORS = [0, 255, 0]
	LED_PULSE_WAVELENGTH = 2.5

	# General
	UPDATE_RATE = 0.1               # Delay after each run_mood() iteration


	def __init__(self, robot_name):
		self.primary_int = primary_interface(robot_name)
		self.led_intensity = 0.0
		self.led_direction = 1.0
		self.blinking = False
		self.blink_step = 0.0
		self.blink_start = 0.0
		self.driving = False


	def run_mood(self):

		### This method will run ~ (1 / UPDATE_RATE) times per second ###
		start_time = time.time()

		# Pulse LEDs
		self.led_intensity += ((2.0 * mood_relaxed.UPDATE_RATE) / (mood_relaxed.LED_PULSE_WAVELENGTH)) * self.led_direction
		if self.led_intensity >= 1.0 or self.led_intensity <= 0.0:
			self.led_direction *= -1.0

		self.led_intensity = max(0.0, self.led_intensity)
		self.led_intensity = min(1.0, self.led_intensity)
		self.primary_int.lights_rgb = mood_relaxed.LED_COLORS
		self.primary_int.lights_amp = int(100 * self.led_intensity)

		# Decide whether to blink / continue blink
		if self.blinking and time.time() - self.blink_start > mood_relaxed.BLINK_SPEED:
			self.blinking = False
			self.primary_int.eyelid_closure_out = mood_relaxed.AVG_EYELID_DROOP
		elif random.random() < (mood_relaxed.UPDATE_RATE / mood_relaxed.AVG_BLINK_RATE):
			self.blinking = True
			self.blink_start = time.time()
			self.primary_int.eyelid_closure_out = 1.0
		else:
			self.primary_int.eyelid_closure_out = mood_relaxed.AVG_EYELID_DROOP


		# If moving, decide whether to stop.. otherwise, decide whether to start
		if self.driving and random.random() < (mood_relaxed.UPDATE_RATE * (mood_relaxed.AVG_DRIVE_SPEED / mood_relaxed.AVG_LIN_DIST)):
			self.primary_int.stop_moving()
			self.driving = False
		elif random.random() < (mood_relaxed.UPDATE_RATE / mood_relaxed.DRIVE_FREQ):
			linear = random.gauss(mood_relaxed.AVG_DRIVE_SPEED, mood_relaxed.DRIVE_SPEED_SIGMA)
			angular = random.gauss(mood_relaxed.AVG_TURN_SPEED, mood_relaxed.TURN_SPEED_SIGMA)
			self.primary_int.update_body_vel(linear, angular)
			self.driving = True

		# Tail wagging
		self.primary_int.tail = mood_relaxed.TAIL_WAG

		

		while time.time() - start_time < mood_relaxed.UPDATE_RATE:
			pass

		#Move head
		self.primary_int.body_config_speed = mood_lonely.HEAD_SPEED
		self.primary_int.body_config = mood_lonely.HEAD_POSITION



#######################################################################################
#  
# Exited mood is characterized by a high blink rate, no eyelid droop, and lots of
# random movement both of the wheels and neck
#
# Eventually, this mood should include a tendency to chase things and and approach 
# people excitedly (and barking if we want to do audio output)
#
#######################################################################################

class mood_excited:

	# Movement params
	DRIVE_FREQ = 5.0               # Avg time between movements
	AVG_DRIVE_SPEED = 0.3           # Avg linear drive speed
	DRIVE_SPEED_SIGMA = 0.05        # Std Dev of linear drive speed
	AVG_TURN_SPEED = 0.0            # Avg angular drive speed
	TURN_SPEED_SIGMA = 1.0          # Std Dev of angular drive speed
	AVG_LIN_DIST = 1.0              # Avg meters travelled during a movement

	#Head params
	HEAD_POSITION = [0,0,0,0]
	LOOK_PARAM = [0,0,3,5]
	LOOK_POSITION = [0,0,0,0]
	HEAD_SPEED = [10,10,10,10]
	LOOK_FREQ = 10.0

	# Eyelid params
	BLINK_SPEED = 0.1
	AVG_BLINK_RATE = 10.0           # Avg time between blinks
	AVG_EYELID_DROOP = 0.0

	# Tail param
	TAIL_WAG = 0.75

	# LED params
	LED_COLORS = [255, 0, 0]
	LED_PULSE_WAVELENGTH = 1.5

	# General
	UPDATE_RATE = 0.1               # Delay after each run_mood() iteration


	def __init__(self, robot_name):
		self.primary_int = primary_interface(robot_name)
		self.sound_int = sound_interface(robot_name)
		self.led_intensity = 0.0
		self.led_direction = 1.0
		self.blinking = False
		self.blink_step = 0.0
		self.blink_start = 0.0
		self.driving = False
		self.looking = False
		self.look_start = 0.0
		self.sound_int.val = 0

	def run_mood(self):

		### This method will run ~ (1 / UPDATE_RATE) times per second ###
		start_time = time.time()

		# Pulse LEDs
		self.led_intensity += ((2.0 * mood_excited.UPDATE_RATE) / (mood_excited.LED_PULSE_WAVELENGTH)) * self.led_direction
		if self.led_intensity >= 1.0 or self.led_intensity <= 0.0:
			self.led_direction *= -1.0

		self.led_intensity = max(0.0, self.led_intensity)
		self.led_intensity = min(1.0, self.led_intensity)
		self.primary_int.lights_rgb = mood_excited.LED_COLORS
		self.primary_int.lights_amp = int(100 * self.led_intensity)

		# Decide whether to blink / continue blink
		if self.blinking and time.time() - self.blink_start > mood_excited.BLINK_SPEED:
			self.blinking = False
			self.primary_int.eyelid_closure_out = mood_excited.AVG_EYELID_DROOP
		elif random.random() < (mood_excited.UPDATE_RATE / mood_excited.AVG_BLINK_RATE):
			self.blinking = True
			self.blink_start = time.time()
			self.primary_int.eyelid_closure_out = 1.0
		else:
			self.primary_int.eyelid_closure_out = mood_excited.AVG_EYELID_DROOP


		# If moving, decide whether to stop.. otherwise, decide whether to start
		if self.driving and random.random() < (mood_excited.UPDATE_RATE * (mood_excited.AVG_DRIVE_SPEED / mood_excited.AVG_LIN_DIST)):
			self.primary_int.stop_moving()
			self.driving = False
			self.sound_int.val = 1
		elif random.random() < (mood_excited.UPDATE_RATE / mood_excited.DRIVE_FREQ):
			linear = random.gauss(mood_excited.AVG_DRIVE_SPEED, mood_excited.DRIVE_SPEED_SIGMA)
			angular = random.gauss(mood_excited.AVG_TURN_SPEED, mood_excited.TURN_SPEED_SIGMA)
			self.primary_int.update_body_vel(linear, angular)
			self.driving = True
			self.sound_int.val = 1
		# Tail wagging
		self.primary_int.tail = 0 #mood_excited.TAIL_WAG

		# Delay appropriate time 
		while time.time() - start_time < mood_excited.UPDATE_RATE:
			pass

		#Move head
		self.primary_int.body_config_speed = mood_excited.HEAD_SPEED
		self.primary_int.body_config = mood_excited.HEAD_POSITION
		
		# Decide whether to look around
		if self.looking and time.time() - self.look_start > 3:
			self.looking = False
			self.primary_int.body_config = mood_excited.HEAD_POSITION
		elif random.random() < (mood_excited.UPDATE_RATE / mood_excited.LOOK_FREQ):
			mood_excited.LOOK_POSITION[2] = random.uniform(-1* mood_excited.LOOK_PARAM[2],mood_excited.LOOK_PARAM[2])
			mood_excited.LOOK_POSITION[3] = random.uniform(-1* mood_excited.LOOK_PARAM[3],mood_excited.LOOK_PARAM[3])
			self.looking = True
			self.look_start = time.time()
			self.primary_int.body_config = mood_excited.LOOK_POSITION
			time.sleep(1.5)
			self.primary_int.body_config = mood_excited.HEAD_POSITION 
		else:
			self.primary_int.body_config = mood_excited.HEAD_POSITION
		mic_test()


#######################################################################################
#  
# Lonely mood is characterized by droopy eyes, slow blink speed, and very little
# movement. Mood should be triggered by low light, inactivity, and lack of interaction
# with people
#
#######################################################################################

class mood_lonely:


	### NEED TO REPLACE ALL mood_excited with 'mood_lonely' ###

	# Movement params
	DRIVE_FREQ = 10.0               # Avg time between movements
	AVG_DRIVE_SPEED = 0.1           # Avg linear drive speed
	DRIVE_SPEED_SIGMA = 0.05        # Std Dev of linear drive speed
	AVG_TURN_SPEED = 0.0            # Avg angular drive speed
	TURN_SPEED_SIGMA = 1.0          # Std Dev of angular drive speed
	AVG_LIN_DIST = 0.5              # Avg meters travelled during a movement

	#Head params
	HEAD_POSITION = [0,1,0,0]
	HEAD_SPEED = [.5,.5,.5,.5]
	HEAD_SHAKE = 1
	HEAD_NOD = 0


	# Eyelid params
	BLINK_SPEED = 1.5
	AVG_BLINK_RATE = 10.0           # Avg time between blinks
	AVG_EYELID_DROOP = 0.5

	# Tail param
	TAIL_WAG = -1.0

	# LED params
	LED_COLORS = [0, 0, 255]
	LED_PULSE_WAVELENGTH = 3

	# General
	UPDATE_RATE = 0.1               # Delay after each run_mood() iteration


	def __init__(self, robot_name):
		self.primary_int = primary_interface(robot_name)
		self.mic_int = mic_interface(robot_name)
		self.led_intensity = 0.0
		self.led_direction = 1.0
		self.blinking = False
		self.blink_step = 0.0
		self.blink_start = 0.0
		self.driving = False


	def run_mood(self):

		### This method will run ~ (1 / UPDATE_RATE) times per second ###
		start_time = time.time()

		# Pulse LEDs
		self.led_intensity += ((2.0 * mood_lonely.UPDATE_RATE) / (mood_lonely.LED_PULSE_WAVELENGTH)) * self.led_direction
		if self.led_intensity >= 1.0 or self.led_intensity <= 0.0:
			self.led_direction *= -1.0

		self.led_intensity = max(0.0, self.led_intensity)
		self.led_intensity = min(1.0, self.led_intensity)
		self.primary_int.lights_rgb = mood_lonely.LED_COLORS
		self.primary_int.lights_amp = int(100 * self.led_intensity)

		# Decide whether to blink / continue blink
		if self.blinking and time.time() - self.blink_start > mood_lonely.BLINK_SPEED:
			self.blinking = False
			self.primary_int.eyelid_closure_out = mood_lonely.AVG_EYELID_DROOP
		elif random.random() < (mood_lonely.UPDATE_RATE / mood_lonely.AVG_BLINK_RATE):
			self.blinking = True
			self.blink_start = time.time()
			self.primary_int.eyelid_closure_out = 1.0
		else:
			self.primary_int.eyelid_closure_out = mood_lonely.AVG_EYELID_DROOP


		# If moving, decide whether to stop.. otherwise, decide whether to start
		if self.driving and random.random() < (mood_lonely.UPDATE_RATE * (mood_lonely.AVG_DRIVE_SPEED / mood_lonely.AVG_LIN_DIST)):
			self.primary_int.stop_moving()
			self.driving = False
		elif random.random() < (mood_lonely.UPDATE_RATE / mood_lonely.DRIVE_FREQ):
			linear = random.gauss(mood_lonely.AVG_DRIVE_SPEED, mood_lonely.DRIVE_SPEED_SIGMA)
			angular = random.gauss(mood_lonely.AVG_TURN_SPEED, mood_lonely.TURN_SPEED_SIGMA)
			self.primary_int.update_body_vel(linear, angular)
			self.driving = True

		# Tail wagging
		self.primary_int.tail = mood_lonely.TAIL_WAG

		# Delay appropriate time 
		while time.time() - start_time < mood_lonely.UPDATE_RATE:
			pass

		#Move head
		self.primary_int.body_config_speed = mood_lonely.HEAD_SPEED
		self.primary_int.body_config = mood_lonely.HEAD_POSITION
		
	

