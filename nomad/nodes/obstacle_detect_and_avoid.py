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
import random
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
        self.detection_threshold = 0.7
        self.backup_speed = 0.3
        self.turn_speed = 0.5
        self.in_front = False
        self.toss = 0

        rospy.loginfo("Initializing collision_detector node with distance threshold %0.1f" % (self.detection_threshold))

        self.obstacle_detected = Bool()

        self.obstacle_detected.data = True

        # get most recent twist on cmd_vel
        self.cmd_vel1 = rospy.Subscriber('cmd_vel', Twist, self.handleTwistMessage)

        # callback on laser
        self.callback = rospy.Subscriber('scan', LaserScan, self.laser_callback)

        self.filtered_cmd_vel1 = rospy.Publisher('filtered_cmd_vel',Twist, queue_size = 10)

        self.filtered_cmd_vel1.publish(self.twistMessage)

        self.obstaclePublisher = rospy.Publisher('obstacle', Bool, queue_size = 10)

    def handleTwistMessage(self,twistMessage):
        rospy.logdebug("translationX: %f, rotationZ: %f" % (
            twistMessage.linear.x,
            twistMessage.angular.z
            ))

        self.twistMessage = twistMessage

        return

    def laser_callback(self, msg):

        if self.twistMessage.linear.x == 0 and self.twistMessage.angular.y == 0:
            self.filtered_cmd_vel1.publish(self.twistMessage)
            return

        # Post 'avoid' twist message if object is within 1 meter
        # in the specified range.
        # The range values given are attempting to go from -45
        # -> 45 field of view and assume the scan values range from
        # 0 -> 180.

        min_ind = len(msg.ranges)/4
        max_ind = 3*len(msg.ranges)/4

        backup = 0

        for scan in msg.ranges[0:min_ind]:
            if (scan < self.detection_threshold):
                backup = 1

        for scan in msg.ranges[max_ind:]:
            if (scan < self.detection_threshold):
                backup = 1

        for scan in msg.ranges[min_ind:max_ind]:
            if (scan < self.detection_threshold):
                self.obstacle_detected.data = True
                self.obstaclePublisher.publish(self.obstacle_detected)

                # Randomly choose a direction of angular movement
                if not self.in_front:
                    self.toss=random.randint(0,1)

                self.in_front = True

                self.avoidTwistMessage.linear.x = -1.0*self.backup_speed*backup

                if self.toss == 0:
                    self.avoidTwistMessage.angular.z = self.turn_speed
                else:
                    self.avoidTwistMessage.angular.z = -(self.turn_speed)

                self.filtered_cmd_vel1.publish(self.avoidTwistMessage)

                return

        self.in_front = False
        self.filtered_cmd_vel1.publish(self.twistMessage)


if __name__ == '__main__':

    rospy.init_node('obstacle_detect_and_avoid')
    collision_detector = CollisionDetector()
    rospy.spin()

