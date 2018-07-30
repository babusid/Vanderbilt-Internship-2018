from interfaces import*
import rospy

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')
print(type(tester.joint_state))
print(tester.joint_state)