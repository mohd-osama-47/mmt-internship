#!/usr/bin/env python3

import cv2
import os
import numpy as np

def detectAndDisplay(frame):
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # historgram is used to drive up contrast and aid in detecting features:
    frame_gray = cv2.equalizeHist(frame_gray)
    cv2.imshow('Gray Histogram', np.vstack([frame_gray2, frame_gray]))
    
    #-- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in faces:
        frame = cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,255), 2)
        faceROI = frame_gray[y:y+h,x:x+w]
        
        #-- In each face, detect eyes
        eyes = eyes_cascade.detectMultiScale(faceROI)
        for (x2,y2,w2,h2) in eyes:
            eye_center = (x + x2 + w2//2, y + y2 + h2//2)
            radius = int(round((w2 + h2)*0.25))
            frame = cv2.circle(frame, eye_center, radius, (255, 0, 0 ), 4)
    
    cv2.imshow('Capture - Face detection', frame)


cv2_base_dir = os.path.dirname(os.path.abspath(cv2.__file__))

face_cascade_name = os.path.join(cv2_base_dir, 'data/haarcascade_frontalface_default.xml')
eyes_cascade_name = os.path.join(cv2_base_dir, 'data/haarcascade_eye_tree_eyeglasses.xml')
face_cascade = cv2.CascadeClassifier()
eyes_cascade = cv2.CascadeClassifier()

#-- 1. Load the cascades
if not face_cascade.load(face_cascade_name):
    print('--(!)Error loading face cascade')
    exit(0)
if not eyes_cascade.load(eyes_cascade_name):
    print('--(!)Error loading eyes cascade')
    exit(0)

camera_device = 4

#-- 2. Read the video stream
cap = cv2.VideoCapture(camera_device)

if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)

while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break
    detectAndDisplay(frame)
    if cv2.waitKey(10) == ord('q'):
        break
