#!/usr/bin/env python
import particle_class
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from visualization_msgs.msg import Marker, MarkerArray
from numpy import genfromtxt

NUM_P = 100
PART_DIM = 3  # x, y, z
x_range = (0, 3)
y_range = (0, 2)
z_range = (0, 1.5)
# cov_mat = 1.5
# cov_mat = 0.05
cov_mat = 0.05
# tags = np.array(([[0, -0.509, 1.643, 0],
#                   [1, -0.509, 1.23, 0],
#                   [2, -0.277, 1.045, 0],
#                   [3, 0.279, 1.045, 0],
#                   [4, 0.513, 1.045, 0],
#                   [5, 0.513, 1.645, 0],
#                   [6, 0, 0.774, 0],
#                   [7, 0, 0.256, 0],
#                   [8, -0.722, 0, 0],
#                   [9, -0.36, 0, 0],
#                   [10, 0.358, 0, 0],
#                   [11, 0.721, 0, 0],
#                   [12, 0, -0.364, 0],
#                   [13, 0, -0.783, 0],
#                   [14, -0.522, -1.649, 0],
#                   [15, -0.522, -1.236, 0],
#                   [16, -0.278, -1.051, 0],
#                   [17, 0.28, -1.051, 0],
#                   [18, 0.527, -1.051, 0],
#                   [19, 0.527, -1.651, 0]]))
# qx_180 = Quaternion(axis=[1, 0, 0], angle=np.pi)
# qz_90n = Quaternion(axis=[0, 0, 1], angle=-np.pi / 2)
#
# dreh = qx_180 * qz_90n
# for i in range(tags.shape[0]):
#     tags[i, 1:5] = dreh.rotate(tags[i, 1:5]) + np.asarray([1.61, 0.861, 1.4])
# print(tags)
# tags = np.array(([[ 0. ,         3.23831319,  0.36636964,  1.31201144  ],
#  [ 1.   ,       2.82066553,  0.36380847,  1.30318445],
#  [ 2.    ,      2.63517697,  0.58942749,  1.30350728],
#  [ 3.     ,     2.63489789,  1.14710925,  1.30307401],
#  [ 4.     ,     2.81139968,  1.37975452,  1.30343474],
#  [ 5.     ,     3.23084454,  1.38791825,  1.31764983],
#  [ 6.     ,     2.36160292,  0.86320916,  1.30411645],
#  [ 7.     ,     1.84235159,  0.85791936,  1.30362097],
#  [ 8.     ,     1.59350598,  0.13490855,  1.30283129],
#  [ 9.     ,     1.59050894,  0.49519434,  1.3040148],
#  [10.     ,     1.58441478,  1.21770259,  1.30707124],
#  [11.     ,     1.57988579,  1.58039629,  1.30724808],
#  [12.     ,     1.3243854 ,  0.85630028,  1.30635755],
#  [13.     ,     0.80354831,  0.85595953,  1.30862898],
#  [14.     ,    -0.05638245,  0.31113967,  1.3248386],
#  [15.     ,     0.35658294,  0.32651376,  1.30891262],
#  [16.     ,     0.53907469,  0.57205991,  1.30846051],
#  [17.     ,     0.53355578,  1.131107  ,  1.31822385],
#  [18.     ,     0.34682712,  1.36617877,  1.32839176],
#  [19.     ,    -0.06760727,  1.35470647,  1.33599061]]))
tags = genfromtxt('calibration.csv', delimiter=',')
tags = tags[:, 0:4]
tags[:, 1] += 0.3
# print(tags)
rviz = False


def yaw_pitch_roll_to_quat(yaw, pitch, roll):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    return (Quaternion(x=cy * cp * sr - sy * sp * cr, y=sy * cp * sr + cy * sp * cr, z=sy * cp * cr - cy * sp * sr,
                       w=cy * cp * cr + sy * sp * sr))


def callback(msg, tmp_list):
    """"""
    [particle_filter, publisher_position, publisher_mavros, publisher_particles, broadcaster, publisher_marker] = tmp_list

    # particle filter algorithm
    particle_filter.predict()  # move particles

    # get length of message
    num_meas = len(msg.detections)
    orientation_yaw_pitch_roll = np.zeros((num_meas, 3))
    # if new measurement: update particles
    if num_meas >= 1:
        measurements = np.zeros((num_meas, 1 + PART_DIM))
        # get data from topic /tag_detection
        if rviz == True:
            markerArray = MarkerArray()
        for i, tag in enumerate(msg.detections):
            tag_id = int(tag.id[0])
            distance = np.array(
                ([tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z]))
            measurements[i, 0] = np.linalg.norm(distance)
            tmpquat = Quaternion(w=tag.pose.pose.pose.orientation.w, x=tag.pose.pose.pose.orientation.x,
                                 y=tag.pose.pose.pose.orientation.y, z=tag.pose.pose.pose.orientation.z)

            orientation_yaw_pitch_roll[i, :] = tmpquat.inverse.yaw_pitch_roll
            index = np.where(tags[:, 0] == tag_id)

            measurements[i, 1:4] = tags[index, 1:4]
            if rviz == True:
                marker = Marker()
                marker.header.frame_id = "global_tank"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = measurements[i, 0] * 2  # r*2 of distance to camera from tag_14
                marker.scale.y = measurements[i, 0] * 2
                marker.scale.z = measurements[i, 0] * 2
                marker.color.g = 1
                marker.color.a = 0.1  # transparency
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = tags[index, 1][0]  # x
                marker.pose.position.y = tags[index, 2][0]  # y
                marker.pose.position.z = tags[index, 3][0]  # z
                markerArray.markers.append(marker)
        if rviz == True:
            # print(len(markerArray.markers))
            publisher_marker.publish(markerArray)
            # print(index)
        particle_filter.update(measurements)

        # print "reale messungen: " + str(measurements)
    yaw = np.mean(orientation_yaw_pitch_roll[:,0])
    pitch = np.mean(orientation_yaw_pitch_roll[:, 1])
    roll = np.mean(orientation_yaw_pitch_roll[:, 2])
    print(yaw*180/np.pi      )
    estimated_orientation = yaw_pitch_roll_to_quat(yaw, 0, 0)

    # calculate position as mean of particle positions
    estimated_position = particle_filter.estimate()

    x_mean = estimated_position[0] * 1000
    y_mean = estimated_position[1] * 1000
    z_mean = estimated_position[2] * 1000

    # publish estimated_pose
    position = PoseStamped()
    position.header.stamp = rospy.Time.now()
    position.header.frame_id = "global_tank"
    position.pose.position.x = x_mean/1000
    position.pose.position.y = y_mean/1000
    position.pose.position.z = z_mean/1000
    publisher_position.publish(position)

    # publish estimated_pose in mavros to /mavros/vision_pose/pose
    # this pose needs to be in ENU
    mavros_position = PoseStamped()
    mavros_position.header.stamp = rospy.Time.now()
    mavros_position.header.frame_id = "map"
    mavros_position.pose.position.x = y_mean/1000
    mavros_position.pose.position.y = x_mean/1000
    mavros_position.pose.position.z = - z_mean/1000

    mavros_position.pose.orientation.w = estimated_orientation[0]
    mavros_position.pose.orientation.x = estimated_orientation[1]
    mavros_position.pose.orientation.y = estimated_orientation[2]
    mavros_position.pose.orientation.z = estimated_orientation[3]

    publisher_mavros.publish(mavros_position)

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
        pose_array.header.frame_id = "global_tank"
        for i in range(num_meas):
            print(orientation_yaw_pitch_roll[i, 0] * 180 / np.pi, orientation_yaw_pitch_roll[i, 1] * 180 / np.pi,
                  orientation_yaw_pitch_roll[i, 2] * 180 / np.pi)
        print("done")
        for i in range(particle_filter.NUM_P):
            pose = Pose()
            pose.position.x = particle_filter.particles[i, 0]
            pose.position.y = particle_filter.particles[i, 1]
            pose.position.z = particle_filter.particles[i, 2]
            # pose.orientation.x =
            # pose.orientation.y =
            # pose.orientation.z =
            # pose.orientation.w =
            pose_array.poses.append(pose)

        publisher_particles.publish(pose_array)
        # add spheres to rviz

        # print pose_array


def main():
    rospy.init_node('particle_filter_node')
    particle_filter = particle_class.ParticleFilter(NUM_P, PART_DIM, x_range, y_range, z_range, cov_mat)
    publisher_position = rospy.Publisher('estimated_pose', PoseStamped, queue_size=1)
    publisher_mavros = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    publisher_particles = rospy.Publisher('particle_poses', PoseArray, queue_size=1)
    publisher_marker = rospy.Publisher('Sphere', MarkerArray, queue_size=1)
    broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback,
                     [particle_filter, publisher_position, publisher_mavros, publisher_particles, broadcaster,
                      publisher_marker], queue_size=1)

    while not rospy.is_shutdown():
        pass


if __name__ == '__main__':
    main()
