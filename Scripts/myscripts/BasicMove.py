import rospy
from secondary import*

if __name__ == "__main__":
    rospy.init_node('DriveTest', anonymous=True)
    miro1 = SecondaryInterface('rob01', 1, 1)
    while True:
        miro1.interrupt()
    rospy.spin()