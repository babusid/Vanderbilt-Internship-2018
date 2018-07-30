from interfaces import *
import rospy
import time
from miro_constants import miro

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')
tester.head_move(MIRO_LIFT_MAX_RAD, MIRO_YAW_MAX_RAD, MIRO_PITCH_MAX_RAD)
time.sleep(2)
tester.head_move(MIRO_LIFT_MIN_RAD, MIRO_YAW_MIN_RAD, MIRO_PITCH_MIN_RAD)
