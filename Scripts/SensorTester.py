from interfaces import*
import rospy

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')
tester.head_move(2, 2, 2)