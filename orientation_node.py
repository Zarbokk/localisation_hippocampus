#!/usr/bin/env python
import particle_class
import numpy as np

import rospy
import tf
from pyquaternion import Quaternion

from geometry_msgs.msg import PoseStamped, PoseArray
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from tf.transformations import rotation_matrix, euler_from_quaternion

NUM_P = 100
PART_DIM = 3  # x, y, z
x_range = (0, 5)
y_range = (0, 5)
z_range = (0, 5)
# cov_mat = 1.5
cov_mat = 0.015
tags = np.array(([[14, 0, 0, 0],
                  [15, 0.7, 0, 0],
                  [16, 0, 0.6, 0],
                  [17, 0.9, 0.6, 0]]))

rviz = True


def callback(msg, list):
    """"""
    [publisher_position, publisher_particles, broadcaster] = list
    # get length of message
    num_meas = len(msg.detections)
    yaw_array = np.zeros(num_meas)
    # if new measurement: update particles
    if num_meas >= 1:

        # get data from topic /tag_detection
        for i, tag in enumerate(msg.detections):
            tag_id = int(tag.id[0])
            distance = Quaternion(w=tag.pose.pose.pose.orientation.w, x=tag.pose.pose.pose.orientation.x,
                                  y=tag.pose.pose.pose.orientation.y, z=tag.pose.pose.pose.orientation.z)
            # distance = Quaternion.random()
            yaw_array[i] = np.asarray(distance.yaw_pitch_roll)[2]
            # print(euler_from_quaternion([distance.x,distance.y]))
            # print(index)
        print(np.mean(yaw_array)*180/np.pi)
        # print "reale messungen: " + str(measurements)

    # calculate position as mean of particle positions
    # publish estimated_pose
    position = PoseStamped()
    position.header.stamp = rospy.Time.now()
    position.header.frame_id = "camera_rect"
    position.pose.orientation.x = 0
    position.pose.orientation.y = 0
    position.pose.orientation.z = 0
    position.pose.orientation.w = 0
    publisher_position.publish(position)

    """
    # publish transform
    broadcaster.sendTransform((estimated_position[0], estimated_position[1], estimated_position[2]),
                              (1.0, 0, 0, 0),
                              rospy.Time.now(),
                              "TestPose",
                              "world")
    """


def main():
    rospy.init_node('particle_filter_node')
    # particle_filter = particle_class.ParticleFilter(NUM_P, PART_DIM, x_range, y_range, z_range, cov_mat)
    publisher_orientation = rospy.Publisher('estimated_orientation', PoseStamped, queue_size=1)
    publisher_particles = rospy.Publisher('particle_orientation', PoseArray, queue_size=1)
    broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback,
                     [publisher_orientation, publisher_particles, broadcaster], queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
