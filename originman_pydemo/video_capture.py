#!/usr/bin/env python3
# encoding:utf-8
import cv2
import time

cap = cv2.VideoCapture(-1)

while True:
    ret, img = cap.read()
    if ret:
        cv2.imshow('img', img)
        key = cv2.waitKey(1)
        if key != -1:
            break
cap.release()
cv2.destroyAllWindows()

