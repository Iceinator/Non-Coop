# -*- coding: utf-8 -*-
"""
Created on Thu Nov 10 14:10:43 2022

@author: chris
"""

import numpy as np
import cv2
import os



path = 'C:\\Users\\chris\\OneDrive - Danmarks Tekniske Universitet\\Master\\Non_coop2\\Data\\image_analysis_data\\Serie_9'
arr = os.listdir(path)

img_test = cv2.imread(path+'\\'+arr[0])
F = 0
cv2.imshow('First Image',img_test)
cv2.waitKey()
#FInder beste crop
while True:
    print('Input start xy for crop')
    x1 = int(input())
    y1 = int(input())
    
    print('\n Input end xy for crop')
    x2 = int(input())
    y2 = int(input())
    
    cropped = img_test[x1:x2, y1:y2]
    
    cv2.imshow('Cropped',cropped)
    cv2.waitKey()
    print('Good?? press 1')
    F = int(input())
    if F==1:
        break

#Cropper alle billeder
newpath = path+'\\Cropped'

os.makedirs(newpath)
for i in range(len(arr)):
    filename = path+'\\'+arr[i]
    img = cv2.imread(filename)
    crop = img[x1:x2, y1:y2]
    
    cv2.imwrite(newpath+'\\'+arr[i],crop)
    
    
    
    
    
    
    

