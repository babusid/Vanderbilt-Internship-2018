import time

import rospy
from interfaces import primary_interface
import math

rospy.init_node('tester', anonymous=True)
tester = primary_interface('rob01')

# tester.head_move(1)
# time.sleep(2)
# tester.head_move(1)
while True:
    tester.head_move(0, .2)
    time.sleep(1)
    x = tester.sonar_range  # left side turn
    time.sleep(1)
    tester.head_move(0, -.2)
    time.sleep(1)
    y = tester.sonar_range  # right side turn
    time.sleep(1)
    tester.head_move()
    print(str(x) + '| leftval')
    print(str(y) + 'rightval')
    if x > 0 and y > 0:
        if x < .1 or y < .1:
            print('straight back')
            tester.drive_straight(-.2)
            time.sleep(2)
            tester.stop_moving()

        elif x > y and (x > .1 or y > .1):
            print('left')
            tester.turn(math.pi)
            time.sleep(.5)
            tester.drive_straight()
            time.sleep(1)
            tester.stop_moving()

        elif y > x and (x > .1 or y > .1):
            print('right')
            tester.turn(-math.pi)
            time.sleep(.5)
            tester.drive_straight()
            time.sleep(1)
            tester.stop_moving()
        elif y == x:

            if y > 0.2 and x > 0.2:
                print('straight forward')
                tester.drive_straight(.2)
                # time.sleep(.5)
                # tester.stop_moving()

            elif y < 0.3 and x < 0.3:
                print('straight back')
                tester.drive_straight(-.2)
                time.sleep(2)
                tester.stop_moving()
    else:
        tester.drive_straight(.2)
        time.sleep(.5)
        tester.stop_moving()
