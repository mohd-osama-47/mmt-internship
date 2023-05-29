#!/usr/bin/venv python3
import numpy as np
import cv2

cam = cv2.VideoCapture(0)

while True:
    ret, frame = cam.read()

    frame_hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    mask =  (frame_hls[:,:,2] >= 50) & \
            (frame_hls[:,:,1] > 0.5 * frame_hls[:,:,2] ) & \
            (frame_hls[:,:,1] < 3 * frame_hls[:,:,2] ) & \
            (frame_hls[:,:,0] <= 14) | \
            (frame_hls[:,:,0] >= 165)

    frame2 = cv2.cvtColor(frame_hls, cv2.COLOR_HLS2BGR)
    output = np.zeros_like(frame2)
    output[mask] = frame2[mask]
    avg_color_b = np.average(output[:,:,0])
    avg_color_g = np.average(output[:,:,1])
    avg_color_r = np.average(output[:,:,2])
    color_window = np.zeros((50,50,3), dtype=np.int8)
    color_window[:,:,0] = avg_color_b
    color_window[:,:,1] = avg_color_g
    color_window[:,:,2] = avg_color_r
    cv2.imshow("Average Color", color_window)
    cv2.imshow("Skin", output)
    if cv2.waitKey(1) == 97:
        break
