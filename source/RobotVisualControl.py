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
        - move_robot(self): will move the robot based on what it gets
        - stop_robot(self): Stops the robot in its tracks.
    '''
    def __init__(self) -> None:
        self.img = None             # frame aquired from the image topic
        self.mask = None            # image mask with green in it
        self.green_zone = None      # image part that has green in it
        self.in_center = -1         # if the object is in the center or not
                                    # key: -1 : not there, 0: left
                                    #       1 : center,    2: right
        assert self.in_center < 3 and self.in_center > -2

        self.bridge = CvBridge()    # CV2 bridge to process images

        # These represent the hsv borders for the color GREEN
        self.hsv_low_green  = (40, 100, 100)
        self.hsv_high_green = (70, 255, 255)

        self.move_command = Twist()

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
        
        # now we can check if the green object is in frame!
        self.green_box_finder()
    
    def green_box_finder(self) -> None:
        '''
        Based on the aquired frame, process to find the green object
        '''
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsv, self.hsv_low_green, self.hsv_high_green)
        self.green_zone = cv2.bitwise_and(self.img, self.img, mask=self.mask)

        # find the center of the detected region

        # THE LAZY METHOD (NOT RECOMMEND!)
        x, y, w, h = cv2.boundingRect(self.mask)
        cv2.rectangle(self.green_zone, (x, y), (x+w, y+h), (0,0,255), 3)
        
        if w < 1:
            # width is too small, no object in frame
            cv2.putText(self.img, ":(", (0,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,255), 2)
            self.in_center = -1
            
        else:
            # now check if it is close to the center (within 10%):
            center = self.img.shape[1]//2
            if (x + w//2 < 0.9*center):
                cv2.putText(self.img, "left", (0,50), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,0), 2)
                self.in_center = 0
            
            elif (x + w//2 > 1.1*center):
                cv2.putText(self.img, "right", (0,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2)
                self.in_center = 2
            
            else:
                cv2.putText(self.img, "nice!", (0,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0), 2)
                self.in_center = 1
        
        self.move_robot()

    def stop_robot(self) -> None:
        '''
        Stops the robot NOW
        '''
        self.move_command.linear.x = 0
        self.move_command.linear.y = 0
        self.move_command.linear.z = 0

        self.move_command.angular.x = 0
        self.move_command.angular.y = 0
        self.move_command.angular.z = 0

        # self.vel_cmd_pub.publish(self.move_command)

    def move_robot(self) -> None:
        '''
        Moves the robot based on where the target object is located
        '''
        if self.in_center == -1:
            # No object detected
            self.stop_robot()
        
        elif self.in_center == 0:
            # object to the left
            self.move_command.angular.z = 15 * 3.1415192 / 180      # rotate at a rate of 15 deg/sec
        
        elif self.in_center == 2:
            # object to the right
            self.move_command.angular.z = -15 * 3.1415192 / 180      # rotate at a rate of 15 deg/sec
        
        else:
            # centered!
            self.stop_robot()

        self.vel_cmd_pub.publish(self.move_command)


if __name__ == '__main__':
    # let us check if the image subscriber works!
    
    rospy.init_node('TestCamera')       # DO NOT FORGET TO INITIALIZE A NODE!
    mlk = Mulekhia()

    while not rospy.is_shutdown():
        if mlk.img is not None:
            # cv2.imshow('ImageTopic',np.vstack([mlk.img, mlk.green_zone]))
            cv2.imshow('ImageTopic',mlk.img)
        if cv2.waitKey(1) == ord('q'):
            break
