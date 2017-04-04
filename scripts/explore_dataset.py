#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import rosbag
from tabulate import tabulate
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import cv2
import cv_bridge
import math
import tf
from tf.msg import tfMessage
import abc
import numpy as np 
import numpy.matlib

kSemimajorAxis = 6378137
kSemiminorAxis = 6356752.3142
kFirstEccentricitySquared = 6.69437999014 * 0.001
kSecondEccentricitySquared = 6.73949674228 * 0.001
kFlattening = 1 / 298.257223563


def geodetic_to_pose(geodetic):
    lat_rad = degree_to_rad(geodetic.latitude)
    lon_rad = degree_to_rad(geodetic.longitude)
    xi = math.sqrt(1 - kFirstEccentricitySquared * math.sin(lat_rad) * math.sin(lat_rad))
    x = (kSemimajorAxis / xi + geodetic.altitude) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (kSemimajorAxis / xi + geodetic.altitude) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * math.sin(lat_rad)
    return x, y, z


def show_topic_detail(file_path):
    bag = rosbag.Bag(file_path)
    type_and_topic_info = bag.get_type_and_topic_info()
    print
    print "## " + file_path
    print
    headers = ['Topic', 'Msg type', 'Msg count', 'frequency', 'connections']

    table = []
    for topic in type_and_topic_info.topics.keys():
        table.append([topic,
                      type_and_topic_info.topics[topic].msg_type,
                      type_and_topic_info.topics[topic].message_count,
                      type_and_topic_info.topics[topic].frequency,
                      type_and_topic_info.topics[topic].connections])
    print tabulate(table, headers=headers, tablefmt='orgtbl').replace('+', '|')
    bag.close()


def show_image(msg):
    bridge = cv_bridge.CvBridge()
    img = bridge.imgmsg_to_cv2(msg)
    cv2.imshow("0", img)
    cv2.waitKey(1)


def degree_to_rad(degree):
    return degree * math.pi / 180.0


class ExploreDataset():

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tfMessage, queue_size=10)
        rospy.Subscriber("/image_raw", Image, show_image)
        rospy.Subscriber("/gps/rtkfix", Odometry, callback=self.capture_vehicle_callback)
        rospy.Subscriber("/obs1/gps/rtkfix", Odometry, callback=self.obs_vehicle_callback)
        # rospy.Subscriber("/tf", tfMessage, callback=self.tf_message_callback)


    def publish_tf_from_rtkfix(self, rtk_msg, frame_name):
        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rtk_msg.header.stamp
        t.child_frame_id = frame_name
        t.transform.translation.x = rtk_msg.pose.pose.position.x
        t.transform.translation.y = rtk_msg.pose.pose.position.y
        t.transform.translation.z = rtk_msg.pose.pose.position.z
        t.transform.rotation.w = 1

        # the orientation is not valid, as the dataset's readme.md said.

        t.transform.rotation.x = rtk_msg.pose.pose.orientation.x
        t.transform.rotation.y = rtk_msg.pose.pose.orientation.y
        t.transform.rotation.z = rtk_msg.pose.pose.orientation.z
        t.transform.rotation.w = rtk_msg.pose.pose.orientation.w
        tanTheta = t.transform.rotation.y / t.transform.rotation.x
        Arctan = np.arctan(tanTheta)
        t.transform.rotation.angle = Arctan/np.pi*360

        tfm = tf.msg.tfMessage([t])
        self.pub_tf.publish(tfm)

    def obs_vehicle_callback(self, msg):
        self.publish_tf_from_rtkfix(msg, "obs1_gps_link")

    def capture_vehicle_callback(self, msg):
        self.publish_tf_from_rtkfix(msg, "base_link")


if __name__ == '__main__':
    rospy.init_node("read_dataset")
    bag_file_path = rospy.get_param('/bag_filename')
    print "*" * 20
    print bag_file_path

    explore_dataset = ExploreDataset()
    show_topic_detail(bag_file_path)

    rospy.spin()
