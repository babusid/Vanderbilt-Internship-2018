import rospy
from secondary import*

if __name__ == "__main__":
    rospy.init_node('Demo', anonymous=True)
    miro1 = SecondaryInterface('rob01', 1, 1)




    rospy.spin()
