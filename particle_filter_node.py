#!/usr/bin/env python
import particle_class
import numpy as np

import pyquaternion as Quaternion
import rospy
import tf

from geometry_msgs.msg import *
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection

NUM_P = 200
PART_DIM = 3  # x, y, z
x_range = (0, 5)
y_range = (0, 5)
z_range = (0, 5)
# cov_mat = 1.5
cov_mat = 0.15
tags = np.array(([[14, 0, 0, 0],
                  [15, 0.7, 0, 0],
                  [16, 0, 0.6, 0],
                  [17, 0.9, 0.6, 0]]))

rviz = True


def callback(msg, list):
    """"""
    [particle_filter, publisher_position, publisher_particles, broadcaster] = list

    # particle filter algorithm
    particle_filter.predict()  # move particles

    # get length of message
    num_meas = len(msg.detections)
    # if new measurement: update particles
    if num_meas >= 1:
        measurements = np.zeros((num_meas, 1 + PART_DIM))
        # get data from topic /tag_detection
        for i, tag in enumerate(msg.detections):
            tag_id = int(tag.id[0])
            distance = np.array(
                ([tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z]))
            measurements[i, 0] = np.linalg.norm(distance)

            index = np.where(tags[:, 0] == tag_id)

            measurements[i, 1:4] = tags[index, 1:4]
            # print(index)
        particle_filter.update(measurements)

        # print "reale messungen: " + str(measurements)

    # calculate position as mean of particle positions
    estimated_position = particle_filter.estimate()

    # publish estimated_pose
    position = PoseStamped()
    position.header.stamp = rospy.Time.now()
    position.header.frame_id = "tag_14"
    position.pose.position.x = estimated_position[0]
    position.pose.position.y = estimated_position[1]
    position.pose.position.z = estimated_position[2]
    publisher_position.publish(position)

    """
    # publish transform
    broadcaster.sendTransform((estimated_position[0], estimated_position[1], estimated_position[2]),
                              (1.0, 0, 0, 0),
                              rospy.Time.now(),
                              "TestPose",
                              "world")
    """

    if rviz == True:
        # publish particles as PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "tag_14"

        for i in range(particle_filter.NUM_P):
            pose = Pose()
            pose.position.x = particle_filter.particles[i, 0]
            pose.position.y = particle_filter.particles[i, 1]
            pose.position.z = particle_filter.particles[i, 2]

            pose_array.poses.append(pose)

        publisher_particles.publish(pose_array)
        # print pose_array


def main():
    rospy.init_node('particle_filter_node')
    particle_filter = particle_class.ParticleFilter(NUM_P, PART_DIM, x_range, y_range, z_range, cov_mat)
    publisher_position = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
    publisher_particles = rospy.Publisher('particle_poses', PoseArray, queue_size=1)
    broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback,
                     [particle_filter, publisher_position, publisher_particles, broadcaster], queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
