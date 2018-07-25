from secondary import secondary_interface
from interfaces import*
import rospy
import time
if __name__ == "__main__":
    rospy.init_node('drivetest', anonymous=True)
    miro1 = secondary_interface('rob01')
    rospy.spin()
