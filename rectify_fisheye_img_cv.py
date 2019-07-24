#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

# print(cv2.__version__)
import numpy as np

# from std_msgs.msg import String
# from nav_msgs.msg import Odometry
# from apriltag_ros.msg import AprilTagDetectionArray,AprilTagDetection
rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher('camera/fisheye1/image_rect', Image, queue_size=1)
rate = rospy.Rate(35)

D = np.asarray([-0.0014495550421997905, 0.03796328976750374, -0.03663171827793121, 0.00644681416451931])

K = np.asarray([[283.67401123046875, 0.0, 420.3479919433594],
                [0.0, 284.69140625, 401.9350891113281],
                [0.000000, 0.000000, 1.000000]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (800, 848), cv2.CV_16SC2)
def rectify_callback(data):
    global map1,map2
    # print("test")
    brige = CvBridge()
    try:
        # brige.
        # frame=brige.imgmsg_to_cv2(data, "bgr8")
        frame = brige.imgmsg_to_cv2(data, "passthrough")
        # frame = brige.compressed_imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
        print(e)
    # cv2.imshow("frame_original", frame)
    # cv2.waitKey(1)
    # dist = np.asarray([-0.201626, 0.024907, 0.002174, -0.002007, 0.000000])

    # print(np.asarray(mtx))
    # h, w = frame.shape[:2]
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    # undistorted_img = cv2.omnidir.undistortImage(frame, K, D, 3, newcameramtx)
    img = frame # your image to undistort

    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # cv2.imshow("rectified_image", undistorted_img)
    # message = Image()
    message = brige.cv2_to_imgmsg(undistorted_img, encoding="passthrough")
    message.header = data.header
    # message.
    pub.publish(message)


    rate.sleep()


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('/camera/fisheye1/image_raw', Image, rectify_callback)

    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    listener()
