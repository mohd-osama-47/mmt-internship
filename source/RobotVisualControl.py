#! /usr/bin/env python3

'''
Create a class that controls the robot through visual feedback
from the camera topic!

class is named (Mulekhia)
-> constructor (__init__): subs and pubs
-> detecting the colored object (green_box_finder): -> find if the object is in the center or not
-> movement (move_robot): move the robot according to what we see
-> stop movement(stop_robot): Stops any form of movement to let the robot rest.
'''
# Library imports
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge

# MSGS imports
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Mulekhia:
    '''
    Class that handles turtlebot3 movement based on image recognetion!

    Methods:
        - __init__(self): Creates all necessery subscribers and publishers
        - cv2_callback(self, img:sensor_msgs/Image) callback that switches from Image to cv2 frame
        - green_box_finder(self): callback that handles image processing
        - move_robot(self, cmd:geometry_msgs/Twist): will move the robot based on what it gets
        - stop_robot(self): Stops the robot in its tracks.
    '''
    def __init__(self) -> None:
        self.img = None             # frame aquired from the image topic
        self.in_center = False      # if the object is in the center or not
        self.bridge = CvBridge()    # CV2 bridge to process images 
        self.vel_cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.image_sub  = rospy.Subscriber("/camera/rgb/image_raw", Image, self.cv2_callback)
        print("intialized everything!")
    
    def cv2_callback(self, msg:Image) -> None:
        '''
        Transforms the msg from a ROS msg type to a cv2 compatible frame.
        Params:
            -msg(sensor_msgs/Image): image data from a camera topic
        Returns:
            - None
        '''
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.img = cv2.resize(img, (img.shape[1]//4, img.shape[0]//4))
    


mlk = Mulekhia()
if __name__ == '__main__':
    # let us check if the image subscriber works!
    # DO NOT FORGET TO INITIALIZE A NODE!
    rospy.init_node('TestCamera')
    while not rospy.is_shutdown():
        if mlk.img is not None:
            cv2.imshow('ImageTopic',mlk.img)
        if cv2.waitKey(1) == ord('q'):
            break
