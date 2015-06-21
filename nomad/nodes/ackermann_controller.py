#!/usr/bin/env python

"""

ackermann_basecontroller

- subscribes to the ackermann_cmd topic for AckermannDriveStamped messages
- uses the included values to set the

"""

import serial, time
import roslib; roslib.load_manifest('nomad')
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

maxMetersPerSecond = 0.0
minMetersPerSecond = 0.0
updateHz           = 5.0


class TraxxasController():
    """Controller() - Controls Traxxas chassis through mini SSC II

    """

    maxSteerAngle = 0.7854
    maxSteerControl = 254
    steerScalingFactor = maxSteerControl / (maxSteerAngle * 2)
    maxSpeed = 0.8
    maxSpeedControl = 254
    speedScalingFactor = maxSpeedControl / (maxSpeed * 2)

    syncMarker = chr(255)
    steerServo = 7
    speedServo = 0
    
    fullLeftSteer = 0
    straightSteer = int(maxSteerControl / 2)
    fullRightSteer = maxSteerControl
    
    stopSpeed = 128

    def __init__(self):
        try:
            self.servoController = serial.Serial(
                port = '/dev/ttyUSB0',
                baudrate = 9600,
                timeout = 5
                )
            self.servoController.setRTS(level = True)
            self.servoController.setDTR(level = True)
        
            self.servoController.flushOutput()
        
            time.sleep(1.0)
        
        except:
            print 'Failed to open serial port /dev/ttyUSB0'
            time.sleep(3.0)
            raise Exception
        
        self.stopRover()

        return
    
    def control(self, servo, value):
        self.servoController.write(self.syncMarker)
        self.servoController.write(chr(servo))
        self.servoController.write(chr(value))

        return
    
    def stopRover(self):
        """stopRover() - stop the rover
    
        Set the velocity to zero and center the steering wheels
        """
    
        self.control(self.steerServo, self.straightSteer)
        self.control(self.speedServo, self.stopSpeed)

        return
    
    def handleControlMessage(self, controlMessage):
    
        steerAngle = controlMessage.drive.steering_angle
        velocity = controlMessage.drive.speed

        if steerAngle > self.maxSteerAngle:
            steerAngle = self.maxSteerAngle
        elif steerAngle < -(self.maxSteerAngle):
            steerAngle = -(self.maxSteerAngle)
        steerControl = (steerAngle * -1 + self.maxSteerAngle) * self.steerScalingFactor
        if velocity > self.maxSpeed:
            velocity = self.maxSpeed
        elif velocity < -(self.maxSpeed):
            velocity = -(self.maxSpeed)
        speedControl = (velocity + self.maxSpeed) * self.speedScalingFactor
        rospy.logdebug('Steer angle: %2.1f, Velocity: %2.1f, Steer: %d, Velocity: %d' % (
            steerAngle,
            velocity,
            steerControl,
            speedControl
            ))
        self.control(self.steerServo, int(steerControl))
        self.control(self.speedServo, int(speedControl))

        return


if __name__ == '__main__':
    rospy.init_node(
        name = 'ackermann_controller',
        log_level = rospy.DEBUG
        )
    rospy.loginfo("Initializing ackermann_controller node")

    controller = TraxxasController()
    rospy.on_shutdown(controller.stopRover)

    maxMetersPerSecond      = rospy.get_param("maxMetersPerSecond", 0.6)
    minMetersPerSecond      = rospy.get_param("minMetersPerSecond", 0.1)
    motorUpdateHz           = rospy.get_param("updateHz",          10.0)
    rospy.loginfo("Max m/s: %0.3f, Min m/s: %0.3f, Hz: %0.3f" % (
        maxMetersPerSecond,
        minMetersPerSecond,
        updateHz
        )
        )

    rospy.Subscriber('ackermann_cmd', AckermannDriveStamped, controller.handleControlMessage)

    rate = rospy.Rate(updateHz)
    while not rospy.is_shutdown():
        rate.sleep()

