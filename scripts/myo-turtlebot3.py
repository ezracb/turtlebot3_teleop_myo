#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from turtlebot3_teleop_myo.msg import MyoArm, MyoPose

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0 	#
#    RIGHT          = 1		#
#    Left           = 2		#
# MyoArm.xdir___________________#
#    UNKNOWN        = 0		#
#    X_TOWARD_WRIST = 1		#
#    X_TOWARD_ELBOW = 2		#
# myo_gest UInt8________________#
#    REST           = 1		#
#    FIST           = 2		#
#    WAVE_IN        = 3		#
#    WAVE_OUT       = 4		#
#    FINGERS_SPREAD = 5		#
#    THUMB_TO_PINKY = 6		#
#    UNKNOWN        = 255	#
#################################


if __name__ == '__main__':

    global armState
    global xDirState
    armState = 1;
    rospy.init_node('turtlebot_teleop_myo', anonymous=True)

    tsPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    # set the global arm states
    def setArm(data):
	global armState
	global xDirState
        armState = data.arm
        xDirState = data.xdir
	rospy.sleep(2.0)

    # Use the calibrated Myo gestures to drive the turtlebot
    def drive(gest):
    	rospy.loginfo(gest.pose)
        if gest.pose == 2: #FIST
	    rospy.loginfo("stop")
	    tsPub.publish(Twist(Vector3(0.0, 0, 0), Vector3(0, 0, 0)))
        elif gest.pose == 3 and armState == 1: #WAVE_IN, RIGHT arm
	    rospy.loginfo("left")
	    tsPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0)))
        elif gest.pose == 3 and armState == 2: #WAVE_IN, LEFT arm
	    rospy.loginfo("right")
	    tsPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -1.0)))
        elif gest.pose == 4 and armState == 1: #WAVE_OUT, RIGHT arm
	    rospy.loginfo("right")
	    tsPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -1.0)))
        elif gest.pose == 4 and armState == 2: #WAVE_OUT, LEFT arm
	    rospy.loginfo("left")
	    tsPub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 1.0)))
        elif gest.pose == 5: #FINGERS_SPREAD
	    rospy.loginfo("forward")
	    tsPub.publish(Twist(Vector3(1.0, 0, 0), Vector3(0, 0, 0)))
        elif gest.pose == 6: #DOUBLE_TAP
	    rospy.loginfo("backward")
	    tsPub.publish(Twist(Vector3(-1.0, 0, 0), Vector3(0, 0, 0)))

    rospy.Subscriber("myo_raw/myo_arm", MyoArm, setArm)
    rospy.Subscriber("myo_raw/myo_gest", MyoPose, drive)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
