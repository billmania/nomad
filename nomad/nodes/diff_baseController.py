#!/usr/bin/env python

"""

baseController

- subscribes to the lmotor and rmotor topics
- uses the values received to drive each motor

listens to:
 /lmotor
 /rmotor

publishes to:

The message on the motor topic is std_msgs/Float32, whose
data element is interpreted as the number of meters per
second at which the wheel should be rotated. The sign of
that value is interpreted as the direction to rotate
the wheel.

"""

import roslib; roslib.load_manifest('nomad')
import rospy
from std_msgs.msg import Float32

from PhidgetMotorController import PhidgetMotorController

motorController = None

leftVelocity = 0.0
rightVelocity = 0.0

maxMetersPerSecond = 0.0
minMetersPerSecond = 0.0
motorUpdateHz      = 0.0

def handleRightMotorMessage(rightMotorMessage):
    global rightVelocity

    if -(minMetersPerSecond) < rightMotorMessage.data < minMetersPerSecond:
        rightVelocity = 0
        return
    elif rightMotorMessage.data >= minMetersPerSecond:
        rightVelocity = int(rightMotorMessage.data / maxMetersPerSecond * 100)
        if rightVelocity > 100:
            rightVelocity = 100
    else:
        rightVelocity = int(rightMotorMessage.data / maxMetersPerSecond * 100)
        if rightVelocity < -100:
            rightVelocity = -100

    return

def handleLeftMotorMessage(leftMotorMessage):
    global leftVelocity

    if -(minMetersPerSecond) < leftMotorMessage.data < minMetersPerSecond:
        leftVelocity = 0
        return
    elif leftMotorMessage.data >= minMetersPerSecond:
        leftVelocity = int(leftMotorMessage.data / maxMetersPerSecond * 100)
        if leftVelocity > 100:
            leftVelocity = 100
    else:
        leftVelocity = int(leftMotorMessage.data / maxMetersPerSecond * 100)
        if leftVelocity < -100:
            leftVelocity = -100

    return

if __name__ == '__main__':
    rospy.init_node(
        name = 'diff_baseController',
        log_level = rospy.DEBUG
        )
    rospy.loginfo("Initializing diff_baseController node")

    maxMetersPerSecond      = rospy.get_param("maxMetersPerSecond", 0.6)
    minMetersPerSecond      = rospy.get_param("minMetersPerSecond", 0.1)
    motorUpdateHz           = rospy.get_param("motorUpdateHz",     10.0)
    rightMotorDirectionSign = rospy.get_param("rightMotorDirectionSign", 1)
    leftMotorDirectionSign  = rospy.get_param("leftMotorDirectionSign", -1)
    leftMotorId             = rospy.get_param("leftMotorId", 0)
    rightMotorId            = rospy.get_param("rightMotorId", 1)
    rospy.loginfo("Max m/s: %0.3f, Min m/s: %0.3f, Hz: %0.3f" % (
        maxMetersPerSecond,
        minMetersPerSecond,
        motorUpdateHz
        )
        )

    motorController = PhidgetMotorController(
        leftMotorId,
        rightMotorId,
        leftMotorDirectionSign,
        rightMotorDirectionSign
        )
    motorController.setDefaultSpeed(75.0)

    rospy.Subscriber('lmotor', Float32, handleLeftMotorMessage)
    rospy.Subscriber('rmotor', Float32, handleRightMotorMessage)

    rate = rospy.Rate(motorUpdateHz)
    while not rospy.is_shutdown():
        motorController.motorsDirect(
            leftVelocity,
            rightVelocity
            )
        rate.sleep()

    motorController.motorsDirect(0.0, 0.0)

