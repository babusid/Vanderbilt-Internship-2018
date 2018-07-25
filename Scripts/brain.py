

#-------------------------------------------------------------------------------------#
#
#
#
# Created by Jacob Gloudemans and James Zhu
# 3/19/2018
#
#-------------------------------------------------------------------------------------#
#-------------------------------------------------------------------------------------#


import rospy
from flags import *
from moods import *


class brain:

	class __brain:

		def __init__(self, robot_name):

			# Inititalize each flag into a dictionary
			self.flags = {
				"cliff detected": flag_cliff_detected(robot_name),
				"object detected": flag_obj_detected(robot_name),
				"petting detected": flag_petting_detected(robot_name),
				"tennis ball detected": flag_tennis_ball_detected(robot_name),
				"low light": flag_low_light(robot_name),
				"noise": flag_audio(robot_name)}

			# Initialize each available mood into a dictionary
			self.moods = {
				"relaxed": mood_relaxed(robot_name),
				"excited": mood_excited(robot_name),
				"lonely" : mood_lonely(robot_name)}

			self.current_mood = self.moods["lonely"] #relaxed
			self.robot_name = robot_name

		def activate(self):
			rospy.init_node(self.robot_name + "s_brain", anonymous=True)
			start_time = rospy.get_rostime()

			while not rospy.is_shutdown():

				self.current_mood.run_mood()
				self.update_mood()

				# debugging line to print status of flags periodically
				if (rospy.get_rostime() - start_time).to_sec() > 5.0:
					self.print_flags()
					start_time = rospy.get_rostime()


		def update_mood(self):

			if self.flags["tennis ball detected"].value:
				self.current_mood = self.moods["excited"]
				print "excited!"

			elif self.flags["petting detected"].value:
				self.current_mood = self.moods["excited"] #relaxed
				print "relaxed"

			#elif self.flags["low light"].value:
				#self.current_mood = self.moods["lonely"]
				#print "lonely :("


		def print_flags(self):
			for flag in self.flags:
				print self.flags[flag]
			print ""

	##########################################################################################
	# Stuff below here ensures that only one instance of this class is created for any robot #
	##########################################################################################

	instances = {}

	def __init__(self, robot_name):

		# Keeps track of which __miro_robot instance this object cares about
		self.robot_name = robot_name 

		# if instance already exists for robot do nothing, otherwise make a new one
		if robot_name in brain.instances:
			pass
		else:
			brain.instances[robot_name] = brain.__brain(robot_name)


	def __getattr__(self, attr):
		return getattr(brain.instances[self.robot_name], attr)

	def __setattr__(self, attr, value):
		if attr == "robot_name":
			self.__dict__["robot_name"] = value
		else:
			setattr(brain.instances[self.robot_name], attr, value)



if __name__ == "__main__":
	new_brain = brain("rob01")
	new_brain.activate()




