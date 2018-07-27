from interfaces import*
import rospy

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')
while True:
    print(tester.joint_state)