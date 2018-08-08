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
    tester.head_move(0, .2)  # turn to left side
    time.sleep(.2)
    x = tester.sonar_range  # left side turn range value
    time.sleep(.2)
    tester.head_move()  # set to middle
    time.sleep(.2)
    tester.head_move(0, -.2)  # turn to right side4
    time.sleep(.2)
    y = tester.sonar_range  # right side turn range value
    time.sleep(.2)
    z = (x + y)/2 # average distance from both sides
    tester.head_move()  # set back to middle
    print(str(x) + '| leftval')
    print(str(y) + '| rightval')
    if x > 0 and y > 0:  # make sure its registering some value

        if z < .3:  # its too close; get away
            print('straight back')
            tester.tail_move(-1)
            tester.drive_straight(-.2)
            time.sleep(2)
            tester.stop_moving()
            tester.tail_move(0)

        else:  # nothing too close

            if x > y:  # right side is closer than left; move left
                print('left')
                tester.turn(math.pi)
                time.sleep(.25)
                tester.drive_straight()
                time.sleep(1)
                tester.stop_moving()

            elif y > x:  # left side is closer than right; move right
                print('right')
                tester.turn(-math.pi)
                time.sleep(.25)
                tester.drive_straight()
                time.sleep(1)
                tester.stop_moving()

            elif y == x:  # both sides are equidistant
                print('straight forward')
                tester.drive_straight(.2)
                time.sleep(.5)
                tester.stop_moving()

    else:  # if nothing registers, move until something does
        tester.drive_straight(.2)
        time.sleep(.5)
        tester.stop_moving()
