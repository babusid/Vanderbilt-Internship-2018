import rospy

from miro_msgs.msg import bridge_stream , bridge_config
from interfaces import primary_interface #  sound_interface

class sound_test:

	def __init__(self):

		# Main publisher
		root = "/miro/" + "rob01" + "/bridge"
		self.cmd_out = rospy.Publisher(root + "/stream", bridge_stream, queue_size=1)
		self.primary_int = primary_interface("rob01")
	

	def data_in(self):

		start_time = rospy.get_rostime()

		#output a sound with index between 1 and 3 (bark, pant, whimper)
		while (rospy.get_rostime() - start_time).to_sec() < .3:
			cmd = bridge_stream()
			cmd.sound_index_P3 = 3
			self.cmd_out.publish(cmd)
			
		
if __name__ == "__main__":

	rospy.init_node("sound_test", anonymous=True)
	behavior=sound_test()
	behavior.data_in()
