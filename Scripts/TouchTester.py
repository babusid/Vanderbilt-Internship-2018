from interfaces import*
import rospy

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')
while True:
    if 1 in tester.touch_body:
        print('activate')
    else:
        print("deactivate")