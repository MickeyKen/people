#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import tf
from geometry_msgs.msg import Quaternion

SEARCH_RADIUS = 0.25

def callback(leg, people, area):
    # rospy.loginfo("leg timestamp: %d ns" % leg.header.stamp.to_nsec())
    # rospy.loginfo("people timestamp: %d ns" % people.header.stamp.to_nsec())
    # rospy.loginfo("area timestamp: %d ns" % area.header.stamp.to_nsec())

    center_x = area.point.x
    center_y = area.point.y
    print ("X: ", center_x, "Y: ", center_y)

    target_name = ""
    target_diff = SEARCH_RADIUS

    posearray_msg = PoseStamped()


    for p in people.people:
        if p.pos.y > 0.0:
            diff = math.hypot(center_x - p.pos.x, center_y - p.pos.y)
            print ("postition: ", p.pos.x, p.pos.y)
            if diff < SEARCH_RADIUS and diff < target_diff:
                target_diff = diff
                target_name = p.object_id
                posearray_msg.pose.position.x = p.pos.x
                posearray_msg.pose.position.y = p.pos.y


    if target_name:
        leg_name0 = target_name.split('|')[0]
        leg_name1 = target_name.split('|')[1]
        print ("name: ",leg_name0, leg_name1)

        legs = [[l.pos.x, l.pos.y] for l in leg.people if l.object_id == leg_name0 or l.object_id == leg_name1]

        print ("length: ",len(legs))

        if len(legs) == 2:

            rad = math.atan2(legs[1][1]-legs[0][1], legs[1][0]-legs[0][0])
            deg = math.degrees(rad)
            deg1 = (deg + 90) % 360
            deg2 = (deg + 270) % 360
            if deg1 > 180:
                q = euler_to_quaternion(deg1)
                posearray_msg.pose.orientation.x = q.x
                posearray_msg.pose.orientation.y = q.y
                posearray_msg.pose.orientation.z = q.z
                posearray_msg.pose.orientation.w = q.w
            elif deg2 > 180:
                q = euler_to_quaternion(deg2)
                posearray_msg.pose.orientation.x = q.x
                posearray_msg.pose.orientation.y = q.y
                posearray_msg.pose.orientation.z = q.z
                posearray_msg.pose.orientation.w = q.w
            else:
                posearray_msg.pose.orientation.x = 0.0
                posearray_msg.pose.orientation.y = 0.0
                posearray_msg.pose.orientation.z = 0.0
                posearray_msg.pose.orientation.w = 1.0



            posearray_msg.header.frame_id = "/base_scan"
            posearray_msg.header.stamp = rospy.Time.now()

            pose_pub.publish(posearray_msg)


    else:
        pass

def euler_to_quaternion(yaw):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw))
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

rospy.init_node('filetering_leg_detector_node')

# pub = rospy.Publisher("/output", PoseStamped, queue_size=1)
pose_pub = rospy.Publisher("/filtering/scan/detect_leg_person", PoseStamped, queue_size=1)
sub1 = message_filters.Subscriber("leg_tracker_measurements", PositionMeasurementArray, queue_size=1)
sub2 = message_filters.Subscriber("people_tracker_measurements", PositionMeasurementArray, queue_size=1)
sub3 = message_filters.Subscriber("target_point", PointStamped, queue_size=1)
ts = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], 10, 1)
ts.registerCallback(callback)

rospy.spin()
