#!/usr/bin/env python

import os
from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer
import numpy as np
import cv2
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



authorizer = None
handler = None
server = None

left_cam_pub = None
right_cam_pub = None
bridge = CvBridge()

pixels_encoding = "mono8"


def publish_image_left_cam(img):
    global left_cam_pub, bridge
    # try:
    left_cam_pub.publish(bridge.cv2_to_imgmsg(img, pixels_encoding))
    # except CvBridgeError as e:
            # print(e)


def publish_image_right_cam(img):
    global right_cam_pub, bridge
    # try:
    right_cam_pub.publish(bridge.cv2_to_imgmsg(img, pixels_encoding))
    # except CvBridgeError as e:
            # print(e)


class CamerasFTPHandler(FTPHandler):

    def on_file_received(self, file):
        print("Hey! File received. Name:")
        print(file)
        img = cv2.imread(file, 0)
        if("img_left" in file):
            print("LLLLLLEEEEEEFFFFFFFTTTTTT")
            cv2.imshow('Left gripper camera',img)
            publish_image_left_cam(img)
    
        elif ("img_right" in file):
            print("RRRRRIIIIIIIIIGGGGGGHHHTTT")
            cv2.imshow('Right gripper camera',img)
            publish_image_right_cam(img)

        cv2.waitKey(1000) # Wait 1000ms before closing the img display window
        cv2.destroyAllWindows()

    def on_incomplete_file_received(self, file):
        print("Hey! Incomplete file received. Name:" + file)
        os.remove(file)



def run_FTP_server():
    global authorizer, handler, server
    # Instantiate a dummy authorizer for managing 'virtual' users
    authorizer = DummyAuthorizer()

    # Define a new user having full r/w permissions and a read-only
    # anonymous user
    authorizer.add_user('right_cam', 'yumiPC', '/home/yumisummer/', perm='elradfmwM')
    authorizer.add_user('left_cam', 'yumiPC', '/home/yumisummer/', perm='elradfmwM')
    # print(os.getcwd())

    # Instantiate FTP handler class
    handler = CamerasFTPHandler
    handler.authorizer = authorizer

    # Define a customized banner (string returned when client connects)
    # handler.banner = "pyftpdlib based ftpd ready."

    # Instantiate FTP server class and listen on 0.0.0.0:2121
    address = ('192.168.125.50', 2121)
    server = FTPServer(address, handler)

    # set a limit for connections
    server.max_cons = 256
    server.max_cons_per_ip = 5

    # start ftp server
    server.serve_forever()


def close_FTP_server():
    global server
    server.close_all()

def main():
    global left_cam_pub, right_cam_pub
    # rospy.init_node("YumiCamerasNode", anonymous=False)
    rospy.on_shutdown(close_FTP_server)

    left_cam_pub = rospy.Publisher("/yumi/left_cam_image", Image, queue_size=1)
    right_cam_pub = rospy.Publisher("/yumi/right_cam_image", Image, queue_size=1)


    # while not rospy.is_shutdown():
    run_FTP_server()


if __name__ == '__main__':
    main()