#!/usr/bin/env python

"""

collision_detector
- monitors the robot's distance sensors, and sends out a collision message
  whenever the distance is small enough

listens to:
 /base_scan

publishes to:
 /collision
 /obstacle

"""

import roslib; roslib.load_manifest('nomad')
import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist


class CollisionDetector():


    def __init__(self):

	self.twistMessage =  Twist()
	self.avoidTwistMessage = Twist()

	self.avoidTwistMessage.linear.x = 0.0
	self.avoidTwistMessage.angular.z = 1.0

        rospy.loginfo("Initializing collision_detector node with distance threshold %0.1f" % (1))

        self.obstacle_detected = Bool()

	self.obstacle_detected.data = True

        self.collision_threshold = 1
	
	# get most recent twist on cmd_vel
	self.cmd_vel1 = rospy.Subscriber('cmd_vel', Twist, self.handleTwistMessage)

	# callback on laser
        self.callback = rospy.Subscriber('scan', LaserScan, self.laser_callback)

	   # check for obstacle
#        self.callback = rospy.Subscriber('obstacle', LaserScan, self.laser_callback)

	   #if none publish cmd_vel to filtered_cmd_vel	

        self.filtered_cmd_vel1 = rospy.Publisher('filtered_cmd_vel',Twist, queue_size = 10) 	

	self.filtered_cmd_vel1.publish(self.twistMessage)

	   #else publish obstacle
	   #publish turn/twist

        self.obstaclePublisher = rospy.Publisher('obstacle', Bool, queue_size = 10)

    def handleTwistMessage(self,twistMessage):
        global lastCmdVelTime

        lastCmdVelTime = rospy.Time.now().to_sec()

        rospy.logdebug("translationX: %f, rotationZ: %f" % (
            twistMessage.linear.x,
            twistMessage.angular.z
            ))

	self.twistMessage = twistMessage
#	self.filtered_cmd_vel1.publish(self.twistMessage)
		
    	return

    def laser_callback(self, msg):

#        detected = False

        for scan in msg.ranges:
            if (scan < 1):
		self.obstacle_detected.data = True	
        	self.obstaclePublisher.publish(self.obstacle_detected)

		self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

	        return                

 	self.filtered_cmd_vel1.publish(self.twistMessage)
	

if __name__ == '__main__':

    rospy.init_node('collision_detector')

    collision_detector = CollisionDetector()

    rospy.spin()

