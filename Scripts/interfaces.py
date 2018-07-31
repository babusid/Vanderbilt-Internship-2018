# Necessary imports
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Temperature, Imu, JointState, Range
from nav_msgs.msg import Odometry
from miro_msgs.msg import platform_control, platform_sensors, platform_mics, bridge_stream
from array import array
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# -------------------------------------------------------------------------------------#
#
# An interface for reading / controlling most sensors / actuators of miro. Subscriber
# recieves sensor data messages from the robot at ~50hz and publishes a control command
# whenever a sensor message is recieved. To read current value of a sensor, simply
# read the platform_interface.[sensor of interest] to get the value. Similarly, to
# command the robot, modify the appropriate class variable. This effectively routes all
# publishing and subscribing to/from the primary robot interfaces through a
# single publisher and subscriber so that each class need not create its own
#
# This class is an 'extended singleton' so there can only be one instance of the class
# for a given robot name. This prevents useless duplication if multiple other programs
# want to access the camera stream from the same robot
#
# Created by Jacob Gloudemans and James Zhu
# 2/15/2018
# Edited by Sidharth Babu
# 6/12/2018
# -------------------------------------------------------------------------------------#
# -------------------------------------------------------------------------------------#


class primary_interface:
    class __primary_interface:

        def __init__(self, robot_name):
            # Main publisher / subscriber
            root = "/miro/" + str(robot_name) + "/platform"
            self.cmd_out = rospy.Publisher(root + "/control", platform_control, queue_size=1)
            self.data_in = rospy.Subscriber(root + "/sensors", platform_sensors, self.data_in, queue_size=1)

            # List to track which flags to update after new data recieved
            self.relevant_flags = []

            # Sensors
            self.battery_voltage = 0.0
            self.temperature = None
            self.accel_head = None
            self.accel_body = None
            self.odometry = None
            self.joint_state = None
            self.eyelid_closure = None
            self.sonar_range = None
            self.light = None
            self.touch_head = [0, 0, 0, 0]
            self.touch_body = [0, 0, 0, 0]
            self.cliff = None

            # Actuators
            self.body_vel = Twist()
            self.body_config = [0.0, 0.0, 0.0, 0.0]
            self.body_config_speed = [0.0, 0.0, 0.0, 0.0]
            self.body_move = Pose2D()
            self.tail = 0.0
            self.ear_rotate = [0.0, 0.0]
            self.eyelid_closure_out = 0.0
            self.blink_time = 0
            self.lights_max_drive = 127
            self.lights_dphase = 0
            self.lights_phase = 0
            self.lights_amp = 0
            self.lights_off = 0
            self.lights_rgb = [0, 0, 0]
            self.sound_index_P1 = 0
            self.sound_index_P2 = 0

            # return variable for pet/pat method
            self.returner = bool
        def data_in(self, data):
            # Update sensor variables
            self.battery_voltage = data.battery_voltage
            self.temperature = data.temperature.temperature
            self.accel_head = data.accel_head  # Imu message type
            self.accel_body = data.accel_body  # Imu message type
            self.odometry = data.odometry  # Odometry message type
            self.joint_state = data.joint_state  # JointState message type
            self.eyelid_closure = data.eyelid_closure
            self.sonar_range = data.sonar_range.range
            self.light = list(array("B", data.light))
            self.touch_head = list(array("B", data.touch_head))
            self.touch_body = list(array("B", data.touch_body))
            self.cliff = list(array("B", data.cliff))

            # Output new control command
            cmd = platform_control()
            cmd.body_vel = self.body_vel
            cmd.body_config = self.body_config
            cmd.body_config_speed = self.body_config_speed
            cmd.body_move = self.body_move
            cmd.tail = self.tail
            cmd.ear_rotate = self.ear_rotate
            cmd.eyelid_closure = self.eyelid_closure_out
            cmd.blink_time = self.blink_time
            cmd.lights_max_drive = self.lights_max_drive
            cmd.lights_dphase = self.lights_dphase
            cmd.lights_phase = self.lights_phase
            cmd.lights_amp = self.lights_amp
            cmd.lights_off = self.lights_off
            cmd.lights_rgb = self.lights_rgb
            cmd.sound_index_P1 = self.sound_index_P1
            cmd.sound_index_P2 = self.sound_index_P2
            self.cmd_out.publish(cmd)

            # Update relevant flags
            for flag in self.relevant_flags:
                flag.update()

        ### Methods to simplify usage of robot ###
        # Updates the body_vel to the specified values (in m/s and rad/sec)
        def update_body_vel(self, linear, angular):
            twist = Twist()
            twist.linear.x = linear * 1000.0
            twist.angular.z = angular
            self.body_vel = twist

        # Sets individual wheels to given speeds in mm/s
        def set_wheel_speeds(self, speed_l, speed_r):
            dr = (speed_l + speed_r) / (2.0 / 1000)
            dtheta = (speed_r - speed_l) / (MIRO_WHEEL_TRACK_MM / 1000)
            self.update_body_vel(dr, dtheta)

        # Stops all robot movement
        def stop_moving(self):
            self.update_body_vel(0, 0)

        # Causes robot to drive straight at given speed in m/s
        def drive_straight(self, speed=0.2):
            self.update_body_vel(speed, 0)

        # Turn at the given velocity in rad/sec
        def turn(self, speed):
            self.update_body_vel(0, speed)

        # Move head joints
        def head_move(self, lift=0.296705973, yaw=0, pitch=0.295833308, speed=-1):
            self.body_config = [0, lift, yaw, pitch]
            self.body_config_speed = [speed, speed, speed, speed]

        def tail_move(self, wag=1):
            self.tail = wag

        def pet_pat(self):
            # this method when called, will continuously update
            # self.returner for the duration of ROS being active
            # to use this method, use the if-else in PrimaryHandler
            # and make it based off of the boolean value
            # it is True when it is being patted
            # it is False when it is being petted

            while not rospy.is_shutdown():

                start_time = rospy.get_rostime()
                self.old_value = 0.0
                flag_pet = False
                flag_pat = False
                self.body_config_speed = [5, 5, 5, 5]

                while not rospy.is_shutdown():

                    if self.touch_body:

                        # find average value of body touch
                        try:
                            self.value = (self.touch_body[0] + 2.0 * self.touch_body[1] + 3.0 *
                                          self.touch_body[2] + 4.0 * self.touch_body[3]) / (
                                             numpy.sum(self.touch_body))
                        except ZeroDivisionError:
                            self.value = 0

                        # test for 2 different types of touching (petting and patting)
                        if abs(self.value - self.old_value) > .5:

                            # No flags activated
                            if flag_pet == False and flag_pat == False:

                                if self.value != 0:
                                    flag_pet = True
                                    flag_pat = False
                                else:
                                    flag_pet = False
                                    flag_pat = True
                                print
                                flag_pet, flag_pat
                                start_time = rospy.get_rostime()
                            # one flag activated
                            elif flag_pet == True:

                                # if next event occurs within 1 sec, initiates response
                                if self.value != 0 and (rospy.get_rostime() - start_time).to_sec() < 1.0:
                                    self.returner = False
                                    flag_pet = False
                                    time.sleep(.5)
                                else:
                                    flag_pet = False
                            elif flag_pat == True:

                                if (rospy.get_rostime() - start_time).to_sec() < 1.0:
                                    self.returner = True
                                    flag_pat = False
                                    time.sleep(.5)
                                else:
                                    flag_pat = False
                        self.old_value = self.value

    ##########################################################################################
    # Stuff below here ensures that only one instance of this class is created for any robot #
    ##########################################################################################

    instances = {}

    def __init__(self, name):

        # Keeps track of which __miro_robot instance this object cares about
        self.robot_name = name

        # if instance already exists for robot do nothing, otherwise make a new one
        if name in primary_interface.instances:
            pass
        else:
            primary_interface.instances[name] = primary_interface.__primary_interface(name)

    def __getattr__(self, attr):
        return getattr(primary_interface.instances[self.robot_name], attr)

    def __setattr__(self, attr, value):
        if attr == "robot_name":
            self.__dict__["robot_name"] = value
        else:
            setattr(primary_interface.instances[self.robot_name], attr, value)


# -------------------------------------------------------------------------------------#
#
# An interface for reading frames from the left and right cameras of the MIRO
# Two class instance variables are kept updated with the most recently recieved
# frame from each of the MIRO's cameras. Images are formatted as cv2 image objects
#
# This class is an 'extended singleton' so there can only be one instance of the class
# for a given robot name. This prevents useless duplication if multiple other programs
# want to access the camera stream from the same robot
#
# Created by Jacob Gloudemans and James Zhu
# 2/20/18
#
# -------------------------------------------------------------------------------------#
# -------------------------------------------------------------------------------------#


class camera_interface:
    class __camera_interface:

        def __init__(self, robot_name):
            root = "/miro/" + str(robot_name) + "/platform"
            self.left_stream = rospy.Subscriber(root + "/caml", Image, self.left_cam_callback, queue_size=1)
            self.right_stream = rospy.Subscriber(root + "/camr", Image, self.right_cam_callback, queue_size=1)
            self.bridge = CvBridge()
            self.cur_img_left = None
            self.cur_img_right = None
            self.cur_img_left_ID = None
            self.cur_img_right_ID = None

            # List to track which flags to update after new data recieved
            self.relevant_flags = []

        def left_cam_callback(self, data):
            new_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cur_img_left = new_img
            self.cur_img_left_ID = "left_" + str(rospy.get_rostime())
            self.update_flags()

        def right_cam_callback(self, data):
            new_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.cur_img_right = new_img
            self.cur_img_left_ID = "right_" + str(rospy.get_rostime())
            self.update_flags()

        def update_flags(self):
            for flag in self.relevant_flags:
                flag.update()

    ##########################################################################################
    # Stuff below here ensures that only one instance of this class is created for any robot #
    ##########################################################################################

    instances = {}

    def __init__(self, name):

        # Keeps track of which __camera_interface instance this object cares about
        self.robot_name = name

        # if instance already exists for robot do nothing, otherwise make a new one
        if name in camera_interface.instances:
            pass
        else:
            camera_interface.instances[name] = camera_interface.__camera_interface(name)

    def __getattr__(self, attr):
        return getattr(camera_interface.instances[self.robot_name], attr)

    def __setattr__(self, attr, value):
        if attr == "robot_name":
            self.__dict__["robot_name"] = value
        else:
            setattr(camera_interface.instances[self.robot_name], attr, value)


# -------------------------------------------------------------------------------------#
#
# An interface for reading microphone data from MIRO by de-interleaving the right and
# left microphone data.
#
# Created by Jacob Gloudemans and James Zhu
# 3/28/18
#
# -------------------------------------------------------------------------------------#
# -------------------------------------------------------------------------------------#

class mic_interface:
    class __mic_interface:

        def __init__(self, robot_name):

            root = "/miro/" + str(robot_name) + "/platform"
            self.data_in = rospy.Subscriber(root + "/mics", platform_mics, self.data_in, queue_size=1)

            # List to track which flags to update after new data recieved
            self.relevant_flags = []
            self.left_mic = []
            self.right_mic = []
            self.even_cnt = True

        def data_in(self, data):

            # reverse interleaving to seperate left and right ear data
            for x in range(4000):
                if self.even_cnt:

                    self.left_mic.append(data.data[x])
                    self.even_cnt = False

                elif not self.even_cnt:

                    self.right_mic.append(data.data[x])
                    self.even_cnt = True
            # Update relevant flags
            for flag in self.relevant_flags:
                flag.update()

    ##########################################################################################
    # Stuff below here ensures that only one instance of this class is created for any robot #
    ##########################################################################################

    instances = {}

    def __init__(self, name):

        # Keeps track of which __mic_interface instance this object cares about
        self.robot_name = name

        # if instance already exists for robot do nothing, otherwise make a new one
        if name in mic_interface.instances:
            pass
        else:
            mic_interface.instances[name] = mic_interface.__mic_interface(name)

    def __getattr__(self, attr):
        return getattr(mic_interface.instances[self.robot_name], attr)

    def __setattr__(self, attr, value):
        if attr == "robot_name":
            self.__dict__["robot_name"] = value
        else:
            setattr(mic_interface.instances[self.robot_name], attr, value)


# -------------------------------------------------------------------------------------#
#
# An interface for producing pre-recorded sounds uploaded onto P3.
#
# Created by Jacob Gloudemans and James Zhu
# 4/13/18
#
# -------------------------------------------------------------------------------------#
# -------------------------------------------------------------------------------------#

class sound_interface:
    class __sound_interface:

        def __init__(self, robot_name):

            self.val = 0
            root = "/miro/" + str(robot_name) + "/platform"
            self.cmd_out = rospy.Publisher(root + "/stream", bridge_stream, queue_size=1)

            # List to track which flags to update after new data recieved
            self.relevant_flags = []

        def data_in(self):

            while not rospy.is_shutdown():
                start_time = rospy.get_rostime()
                # sound_trigger = 0
                while (rospy.get_rostime() - start_time).to_sec() < .3:
                    cmd = bridge_stream()
                    cmd.sound_index_P3 = self.val
                    self.cmd_out.publish(cmd)

    if __name__ == "__main__":
        behavior = sound_interface()
        behavior.data_in()

    ##########################################################################################
    # Stuff below here ensures that only one instance of this class is created for any robot #
    ##########################################################################################

    instances = {}

    def __init__(self, name):

        # Keeps track of which __sound_interface instance this object cares about
        self.robot_name = name

        # if instance already exists for robot do nothing, otherwise make a new one
        if name in sound_interface.instances:
            pass
        else:
            sound_interface.instances[name] = sound_interface.__sound_interface(name)

    def __getattr__(self, attr):
        return getattr(sound_interface.instances[self.robot_name], attr)

    def __setattr__(self, attr, value):
        if attr == "robot_name":
            self.__dict__["robot_name"] = value
        else:
            setattr(sound_interface.instances[self.robot_name], attr, value)
