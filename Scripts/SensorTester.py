import time

import rospy
from interfaces import primary_interface

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')

# tester.head_move(1)
# time.sleep(2)
# tester.head_move(1)
while True:
    print(tester.sonar_range)