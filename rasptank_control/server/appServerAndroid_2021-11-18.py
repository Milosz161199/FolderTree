#!/usr/bin/env python3
# File name   : appServerAndroid.py
# Description : Android Server App
# Author      : Milosz Plutowski
# Date        : 2021/12/19
# Version     : 2.0

import os
import time
from time import ctime
import base64
import RPi.GPIO as GPIO
from socket import *
import move
import manipulatorArmAngle
import ultrasonic
import threading
import json
import cv2
import numpy as np
from threading import Timer
import adafruit_motor.servo
from adafruit_servokit import ServoKit

move.setup()
kit = ServoKit(channels=16)

# SET GPIO PINs for LINE TRACKING
line_pin_right  = 19
line_pin_middle = 16
line_pin_left   = 20

# Value of start servo angles
theta_1 = 90
theta_2 = 130
theta_3 = 130
theta_4 = 90
theta_5 = 90
theta_camera = 78

# BEGIN Commends #
ctrCmd = ["forward", "backward", "right", "left", "forwardRight", "forwardLeft", "backwardRight", "backwardLeft" ,"stop", "speed"]
armCmd = ["theta1up", "theta1down", "theta2up", "theta2down", "theta3up", "theta3down", "theta4up", "theta4down", "grab", "lose", "base", "clean", "stopTheta1"]
policeLED = ["police_on", "police_off"]
lineTrackingCmd = ["tracking_on", "tracking_off"]
systemCmd = ["shutdown", "reboot", "IMG", "followObject_on", "followObject_off"]
# END Commends #

# BEGIN Socket Connection Configuration #
HOST = ""
PORT = 21567
BUFSIZE = 1024

ADDR = (HOST, PORT)
tcpSerSock = socket(AF_INET, SOCK_STREAM)
tcpSerSock.bind(ADDR)
tcpSerSock.listen(5)
# END Socket Connection Configuration #

current_speed = 30
speed_set = 80
setSpeed = False
speedControl = False
maxSpeed = False
direction = 'no'
turn = 'no'

onStart = True
policeLedStart = False
lineTrackingStart = False
followObjectStart = False
segregateStart = False
manipulatorStart = False
cmdManipulator = ' '
based = False
streamON = False
followColorName = 'Yellow'

distanceUltraSensor = 0.0
angleStepStart = True
valOfStepAngle = 2
valOfStepAngle_MAX = 10

# data dictionary
dict_data = {'byte_data': ''}
dict_data_base = {'based': ''}
shape_color_data = [{'name': 'shape', 'value':''},{ 'name':'color', 'value':''}]
line_tracking_sensor_data = {"sensor1":True, "sensor2":False, "sensor3":True, "counter":0}

path_to_img = '/home/pi/rasptank_control/server_2021-12-19/images/my_img.png'
path_to_json = '/home/pi/server/appServer/byte_array.json'
path_to_based_check_json = '/home/pi/server/appServer/based_check.json'

# AutoMode 'memory bits'
turnedRight = 0
turnedLeft = 0
turnedRotationLeft_180 = 0
turnedRotationRight_180 = 0

'''
    @brief checking if the robot is on the black line
           if 'yes' -> write data 'based' to json file
           if 'no' -> write data 'no' to json file
'''
def CheckStartPointBlackLine():
    global based
    left, middle, right = ReadForCheck()
    
    if not left and middle and not right:
        based = True
    else:
        based = False
    
    if based: 
        s = "based"
    else:
        s = 'no'

    dict_data_base['based'] = s
    tmp = dict_data_base
    tmp_json = json.dumps(tmp)

    # save to file
    try:
        file = open(path_to_based_check_json, "w")
        file.write(tmp_json)
    except:
        print("Write Error - RPY")
    finally:
        file.close()
    
'''
    @brief reading the photo and converting it into a string
           and save data to json file
'''
def ReadByteDataFromImage():
    with open(path_to_img, "rb") as img_file:
        my_string = base64.b64encode(img_file.read())

    s = str(my_string)
    s = s[2:len(s)-1]

    dict_data['byte_data'] = s
    tmp = dict_data
    tmp_json = json.dumps(tmp)

    # save to file
    try:
        file = open(path_to_json, "w")
        file.write(tmp_json)
    except:
        print("Write Error - RPY")
    finally:
        file.close()

'''
    @brief openCV in AutoMode - found color and shape by taking one photo 
    @in path - path to image
'''
def FindSpaheAndColorSegregate(path="images/my_img.png"):
    global shape, color 
    shapeFound = False
    colorFound = False
    objCor = 0
    
    cap = cv2.VideoCapture(0)
    _, img = cap.read()
    imgColor = img.copy()
    
    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    imgCanny = cv2.Canny(imgGray, 186, 151)
    kernel = np.ones((3, 3))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    
    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        #print(area)
        if area > 1000:
            cv2.drawContours(img, cnt, -1, (255, 0, 0), 2)
            peri = cv2.arcLength(cnt, True)
            # print(peri)
            shapeFound = True
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 3:
                objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circle"
            else:
                objectType = "None"
            
            shape = objectType
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(img, objectType, (x + w + 5, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 0), 1)

    hsv = cv2.cvtColor(imgColor, cv2.COLOR_BGR2HSV)
    
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])

    red_lower = np.array([136, 87, 111])
    red_upper = np.array([180, 255, 255])

    green_lower = np.array([32, 120, 100])
    green_upper = np.array([85, 242, 130])

    yellow_lower = np.array([19, 107, 89])
    yellow_upper = np.array([35, 255, 255])

    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_result = cv2.bitwise_and(imgColor, imgColor, mask=red_mask)
    yellow_result = cv2.bitwise_and(imgColor, imgColor, mask=yellow_mask)
    green_result = cv2.bitwise_and(imgColor, imgColor, mask=green_mask)
    blue_result = cv2.bitwise_and(imgColor, imgColor, mask=blue_mask)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            color = "Yellow"
            colorFound = True
            cv2.putText(imgColor, "Yellow", (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 255, 0))

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            color = "Red"
            colorFound = True
            cv2.putText(imgColor, "Red", (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255))

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            color = "Green"
            colorFound = True
            cv2.putText(imgColor, "Green", (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0))

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            color = "Blue"
            colorFound = True
            cv2.putText(imgColor, "Blue", (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 0, 0))
    
    if (shapeFound and colorFound):
        shape_color_data[0]['value'] = shape
        shape_color_data[1]['value'] = color
    elif (not shapeFound and colorFound):
        shape_color_data[0]['value'] = '-'
        shape_color_data[1]['value'] = color
    elif (shapeFound and not colorFound):
        shape_color_data[0]['value'] = shape
        shape_color_data[1]['value'] = '-' 
    elif (not shapeFound and not colorFound):
        shape_color_data[0]['value'] = '-'
        shape_color_data[1]['value'] = '-' 

    tmp = shape_color_data
    tmp_json = json.dumps(tmp)
    print(tmp_json)
    # save to file
    try:
        file = open('/home/pi/server/appServer/shape_and_color.json', "w")
        file.write(tmp_json)
    except:
        print("Write Error - RPY")
    finally:
        file.close()
    
    cv2.imwrite(path, img)
    cap.release()

'''
    @brief openCV - found color and shape by taking one photo
    @in shape - wanted shape
    @in colorName - wanted color
    @in path - path to image
    @return shapeFound - (bool) if shape found
    @return coloeFound - (bool) if color found
'''     
def FindSpaheAndColor(shape, colorName, path="images/my_img.png"):
    objCor = 0
    shapeFound = False
    colorFound = False
    cap = cv2.VideoCapture(0)
    _, img = cap.read()
    imgColor = img.copy()
    
    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    imgCanny = cv2.Canny(imgGray, 30, 10)
    kernel = np.ones((3, 3))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    
    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        #print(area)
        if area > 1000:
            #cv2.drawContours(img, cnt, -1, (255, 0, 0), 2)
            peri = cv2.arcLength(cnt, True)
            # print(peri)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 3:
                objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circle"
            else:
                objectType = "None"
            
            if str(objectType) == str(shape):
                shapeFound = True
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.putText(img, objectType, (x + w + 5, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 0), 1)
    
    hsv = cv2.cvtColor(imgColor, cv2.COLOR_BGR2HSV)
    
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])

    red_lower = np.array([136, 87, 111])
    red_upper = np.array([180, 255, 255])

    green_lower = np.array([32, 120, 100])
    green_upper = np.array([85, 242, 130])

    yellow_lower = np.array([19, 107, 89])
    yellow_upper = np.array([35, 255, 255])
       
    if colorName == 'Yellow':
        mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        result = cv2.bitwise_and(imgColor, imgColor, mask=mask)
    elif colorName == 'Red':
        mask = cv2.inRange(hsv, red_lower, red_upper)
        result = cv2.bitwise_and(imgColor, imgColor, mask=mask)
    elif colorName == 'Blue':
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        result = cv2.bitwise_and(imgColor, imgColor, mask=mask)
    elif colorName == 'Green':
        mask = cv2.inRange(hsv, green_lower, green_upper)
        result = cv2.bitwise_and(imgColor, imgColor, mask=mask)
    else:
        mask = cv2.inRange(hsv, green_lower, green_upper)
        result = cv2.bitwise_and(imgColor, imgColor, mask=mask)
        
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            colorFound = True
            cv2.putText(img, colorName, (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 0))
    
    if (shapeFound and colorFound):
        shape_color_data[0]['value'] = shape
        shape_color_data[1]['value'] = colorName
    elif (not shapeFound and colorFound):
        shape_color_data[0]['value'] = '-'
        shape_color_data[1]['value'] = colorName
    elif (shapeFound and not colorFound):
        shape_color_data[0]['value'] = shape
        shape_color_data[1]['value'] = '-' 
    elif (not shapeFound and not colorFound):
        shape_color_data[0]['value'] = '-'
        shape_color_data[1]['value'] = '-' 
    tmp = shape_color_data
    tmp_json = json.dumps(tmp)
    print(tmp_json)
    # save to file
    try:
        file = open('/home/pi/server/appServer/shape_and_color.json', "w")
        file.write(tmp_json)
    except:
        print("Write Error - RPY")
    finally:
        file.close()
    
    cv2.imwrite(path, img)
    cap.release()
     
    return shapeFound, colorFound
 
'''
    @brief openCV in Camera - found color and shape on photo 
    @in path - path to image
'''
def FindSpahesAndColors(path):
    global imageFrame, imgConture, imgCanny
    imageFrame = cv2.imread(path)
    imgConture = imageFrame.copy()
    
    hsv = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    imgGray = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
    imgCanny = cv2.Canny(imgBlur, 30, 13)
    
    kernel = np.ones((3, 3))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    
    getContours(imgDil, imageFrame)
    getColors(hsv, imageFrame)
    
    cv2.destroyAllWindows()
    cv2.imwrite(path_to_img, imageFrame)
    
'''
    @brief - find shapes on image 
    @in img - a photo where we are looking for shapes
    @in imgOut - a photo where we mark the found shapes
'''
def getContours(img, imgOut):  
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000:
            #cv2.drawContours(imgOut, cnt, -1, (255, 255, 0), 1)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 3:
                objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circle"
            else:
                objectType = "None"
            
            cv2.rectangle(imgOut, (x, y), (x + w, y + h), (255, 0, 0), 2)
            if area > 4000:
                cv2.putText(imgOut, objectType, (x + w, y + h), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            else:
                cv2.putText(imgOut, objectType, (x + int(w/2), y + int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

'''
    @brief - find the given shape on image
    @in shape - wanted shape
    @in path - path to image
    @return shapeFound - (bool) if shape found
'''
def getShape(shape, path="images/my_img.png"):
    objCor = 0
    shapeFound = False
    cap = cv2.VideoCapture(0)
    #cap.set(10, 30)
    _, img = cap.read()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (7, 7), 1)
    

    imgCanny = cv2.Canny(imgBlur, 30, 13)
    kernel = np.ones((3, 3))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    
    contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        
        if area > 1000 and area < 9000:
            print(area)
            cv2.drawContours(img, cnt, -1, (255, 255, 0), 2)
            peri = cv2.arcLength(cnt, True)
            # print(peri)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            print(len(approx))
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)

            if objCor == 3:
                objectType = "Triangle"
            elif objCor == 4:
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05:
                    objectType = "Square"
                else:
                    objectType = "Rectangle"
            elif objCor > 4:
                objectType = "Circle"
            else:
                objectType = "None"
            
            if objectType == shape:
                shapeFound = True
                shape = objectType
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.putText(img, objectType, (x + w + 5, y + h + 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 0), 1)
    
                shape_color_data[0]['value'] = shape
                shape_color_data[1]['value'] = '-'
                tmp = shape_color_data
                tmp_json = json.dumps(tmp)
                print(tmp_json)
                # save to file
                try:
                    file = open('/home/pi/server/appServer/shape_and_color.json', "w")
                    file.write(tmp_json)
                except:
                    print("Write Error - RPY")
                finally:
                    file.close()
            else:
                shape_color_data[0]['value'] = '-'
                shape_color_data[1]['value'] = '-'
                tmp = shape_color_data
                tmp_json = json.dumps(tmp)
                print(tmp_json)
                # save to file
                try:
                    file = open('/home/pi/server/appServer/shape_and_color.json', "w")
                    file.write(tmp_json)
                except:
                    print("Write Error - RPY")
                finally:
                    file.close()
    cv2.imwrite(path, img)
    cap.release()
    return shapeFound
      
'''
    @brief - find colors on image
    @in hsv - hsv 
    @in imgColor - a photo where we are looking for colors 
'''
def getColors(hsv, imgColor):
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])

    red_lower = np.array([0, 43, 46])
    red_upper = np.array([10, 255, 255])

    green_lower = np.array([56, 43, 24])
    green_upper = np.array([94, 255, 255])

    yellow_lower = np.array([18, 111, 85])
    yellow_upper = np.array([35, 255, 255])

    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    red_result = cv2.bitwise_and(imgColor, imgColor, mask=red_mask)
    yellow_result = cv2.bitwise_and(imgColor, imgColor, mask=yellow_mask)
    green_result = cv2.bitwise_and(imgColor, imgColor, mask=green_mask)
    blue_result = cv2.bitwise_and(imgColor, imgColor, mask=blue_mask)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 100):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            cv2.putText(imgColor, "Yellow", (x + int(w/2), y + int(h/2) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255))

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 100):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            cv2.putText(imgColor, "Red", (x + int(w/2), y + int(h/2) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255))

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 100):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(imgColor, "Green", (x + int(w/2), y + int(h/2) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255))

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 100):
            x, y, w, h = cv2.boundingRect(contour)
            # imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            cv2.putText(imgColor, "Blue", (x + int(w/2), y + int(h/2) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255))

'''
    @brief - find the given color on image
    @in colorName - wanted color
    @in path - path to image
    @return colorFound - (bool) if color found
'''
def getColor(colorName, path="images/my_img.png"):
    color = '-'
    cap = cv2.VideoCapture(0)
    _, img = cap.read()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    colorFound = False
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])

    red_lower = np.array([136, 87, 111])
    red_upper = np.array([180, 255, 255])

    green_lower = np.array([32, 120, 100])
    green_upper = np.array([85, 242, 130])

    yellow_lower = np.array([19, 107, 89])
    yellow_upper = np.array([35, 255, 255])

    if colorName == 'Yellow':
        mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        result = cv2.bitwise_and(img, img, mask=mask)
    elif colorName == 'Red':
        mask = cv2.inRange(hsv, red_lower, red_upper)
        result = cv2.bitwise_and(img, img, mask=mask)
    elif colorName == 'Blue':
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        result = cv2.bitwise_and(img, img, mask=mask)
    elif colorName == 'Green':
        mask = cv2.inRange(hsv, green_lower, green_upper)
        result = cv2.bitwise_and(img, img, mask=mask)
    else:
        mask = cv2.inRange(hsv, green_lower, green_upper)
        result = cv2.bitwise_and(img, img, mask=mask)
        
    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 1000):
            color = colorName
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x-15, y-15), (x + w + 15, y + h + 15), (255, 0, 255), 2)
            colorFound = True
            cv2.putText(img, colorName, (x + w + 20, y + h),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 0))
        else:
            color = '-'
            
    shape_color_data[0]['value'] = '-'
    shape_color_data[1]['value'] = color
    tmp = shape_color_data
    tmp_json = json.dumps(tmp)
    print(tmp_json)
    # save to file
    try:
        file = open('/home/pi/server/appServer/shape_and_color.json', "w")
        file.write(tmp_json)
    except:
        print("Write Error - RPY")
    finally:
        file.close()                   

    cv2.imwrite(path, img)
    cap.release()
    return colorFound

'''
    @brief - taking photo and save it
    @in w - width
    @in h - wysokość
    @in b - jasność
    @in path - path where the photo is saved
    @in name - image name
'''
def TakeAndWritePhoto(w, h, b, path, name):
    cap = cv2.VideoCapture(0)
    cap.set(3, w)
    cap.set(4, h)
    if b > 100:
        b=100
    elif b < 0:
        b=0
    cap.set(10, b)

    _, img = cap.read()

    path=path+'/'+ name
    #print(path)
    cv2.imwrite(path, img)
    cap.release()

'''
    @brief - Socket Communication
             Reading data form Android App
'''
def SocketCommunication():
    global speedControl, direction, turn, based, path, onStart, frameWidth, frameHeight, cap, segregateStart, policeLedStart, lineTrackingStart, followObjectStart, manipulatorStart, cmdManipulator, followColorName, theta_camera, theta_1, theta_2, theta_3, theta_4, theta_5, speed_set, valOfStepAngle, t, cancelOnes, angleStepStart
    if(onStart):
        os.system("sudo service motion stop")
        manipulatorArmAngle.RotateHead()
        manipulatorArmAngle.servoBase()
        manipulatorArmAngle.clean_all()
        setup_Segregate()
        cap = cv2.VideoCapture(0)
        cap.set(3, frameWidth)
        cap.set(4, frameHeight)
        cap.release()
        
        tmp = line_tracking_sensor_data
        tmp_json = json.dumps(tmp)

        # save to file
        try:
            file = open('/home/pi/server/appServer/line_tracking_sensor.json', "w")
            file.write(tmp_json)
        except:
            print("Write Error - RPY")
        finally:
            file.close()
        
        os.system("sudo python3 LED_green.py")
        time.sleep(0.5)
        os.system("sudo python3 LED_clear.py")
        time.sleep(0.5)
        os.system("sudo python3 LED_green.py")
        time.sleep(0.5)
        os.system("sudo python3 LED_clear.py")
        onStart=False

    while True:
        print("Waiting for connection... ")
        tcpCliSock, addr = tcpSerSock.accept()
        print("...connected from: ", addr)
        data = ""
        try:
            while True:
                data = ""
                data = tcpCliSock.recv(BUFSIZE)
                data = data.decode("utf-8", "ignore")

                data = data[2:len(data)]
                data = data.split(';')
                print(data)
                print("--------------------")
                #print(len(data))

                # NULL DATA
                if len(data) <= 0:
                    break
                if data[0] == 'followObject_on':
                    print("Follow Object start")
                    cap = cv2.VideoCapture(0)
                    cap.set(3, frameWidth)
                    cap.set(4, frameHeight)
                    followObjectStart = True
                    followColorName = str(data[1])
                    print(followColorName)
                    Thread_Follow_Object().start()
                    break
                if data[0] == 'followObject_off':
                    print("Follow Object stop")
                    followObjectStart = False
                    cap.release()
                    break
                if data[0] == 'AutoMode':
                    os.system("sudo python3 LED_clear.py")
                    shapeFound = False
                    colorFound = False
                    
                    if data[1] == 'Shape':
                        shapeFound = getShape(str(data[2]))
                        ReadByteDataFromImage()
                        if shapeFound:
                            os.system("sudo python3 LED_green.py")
                        else:
                            os.system("sudo python3 LED_red.py")
                    elif data[1] == 'Color':
                        colorFound = getColor(str(data[2]))
                        ReadByteDataFromImage()
                        if colorFound:
                            if str(data[2]) == 'Green':
                                os.system("sudo python3 LED_green.py")
                            elif str(data[2]) == 'Red':
                                os.system("sudo python3 LED_red.py")
                            elif str(data[2]) == 'Blue':
                                os.system("sudo python3 LED_blue.py")
                            elif str(data[2]) == 'Yellow':
                                os.system("sudo python3 LED_yellow.py")
                        else:
                            os.system("sudo python3 LED_red.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_clear.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_red.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_clear.py")
                    elif data[1] == 'ShapeColor':
                        shapeFound, colorFound = FindSpaheAndColor(str(data[2]), str(data[3]), path="images/my_img.png")
                        ReadByteDataFromImage()
                        if colorFound:
                            if str(data[3]) == 'Green':
                                os.system("sudo python3 LED_green.py")
                            elif str(data[3]) == 'Red':
                                os.system("sudo python3 LED_red.py")
                            elif str(data[3]) == 'Blue':
                                os.system("sudo python3 LED_blue.py")
                            elif str(data[3]) == 'Yellow':
                                os.system("sudo python3 LED_yellow.py")
                        else:
                            os.system("sudo python3 LED_red.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_clear.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_red.py")
                            time.sleep(0.33)
                            os.system("sudo python3 LED_clear.py")
                    elif data[1] == 'Pick':
                        manipulatorArmAngle.servoGrab()
                    elif data[1] == 'Lose':
                        manipulatorArmAngle.servoLose()
                    elif data[1] == 'Segregate':
                        if based:
                            print("Starting...")
                            segregateStart = True
                            Thread_Segregate().start()
                        else:
                            os.system("sudo python3 LED_red.py")
                    elif data[1] == 'Base':
                        CheckStartPointBlackLine()
                        if based:
                            os.system("sudo python3 LED_green.py")
                        else:
                            os.system("sudo python3 LED_red.py")
                    elif data[1] == 'Stop':
                        os.system("sudo python3 LED_clear.py")
                        segregateStart = False
                    break
                if data[0] == 'motion_on':
                    print("stream on")
                    os.system("sudo service motion start")
                    break
                if data[0] == 'motion_off':
                    print("stream off")
                    os.system("sudo service motion stop")
                    break
                if data[0] == policeLED[0]:
                    print("Police LED - start")
                    policeLedStart = True
                    ''' POLICE LEDs THREAD '''
                    Thread_Led_Police().start()
                    break
                if data[0] == policeLED[1]:
                    print("Police LED - stop")
                    policeLedStart = False
                    os.system("sudo python3 LED_clear.py")
                    break
                if data[0] == lineTrackingCmd[0]:
                    print("Line tracking - start")
                    lineTrackingStart = True
                    ''' LINE TRACKING THREAD '''
                    Thread_Line_Tracking().start()
                    break
                if data[0] == lineTrackingCmd[1]:
                    print("Line tracking - stop")
                    lineTrackingStart = False
                    move.motorStop() # TURN OFF MOTORs
                    break
                if data[0] == ctrCmd[8]:
                    #print("stop motors")
                    direction = 'no'
                    turn = 'no'
                    speedControl = False

                    move.motorStop()
                    break
                if data[0] == ctrCmd[9]:
                    speed_set = int(data[1])
                    #print("Value of speed:" + data[1])
                    break
                if data[0] == ctrCmd[0]:
                    #print("drive forward")
                    speedControl = True
                    direction = 'forward'
                    turn = 'no'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "forward", "no", 0.8)
                    break
                if data[0] == ctrCmd[1]:
                    #print("drive backward")
                    speedControl = True
                    direction = 'backward'
                    turn = 'no'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "backward", "no", 0.8)
                    break
                if data[0] == ctrCmd[2]:
                    #print("turn right")
                    speedControl = True
                    direction = 'no'
                    turn = 'right'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "no", "right", 0.8)
                    break
                if data[0] == ctrCmd[3]:
                    #print("turn left")
                    speedControl = True
                    direction = 'no'
                    turn = 'left'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "no", "left", 0.8)
                    break
                if data[0] == ctrCmd[4]:
                    #print("drive forward-right")
                    speedControl = True
                    direction = 'forward'
                    turn = 'right'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "forward", "right", 0.8)
                    break
                if data[0] == ctrCmd[5]:
                    #print("drive forward-left")
                    speedControl = True
                    direction = 'forward'
                    turn = 'left'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "forward", "left", 0.8)
                    break
                if data[0] == ctrCmd[6]:
                    #print("drive backward-right")
                    speedControl = True
                    direction = 'backward'
                    turn = 'right'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "backward", "right", 0.8)
                    break
                if data[0] == ctrCmd[7]:
                    #print("drive backward-left")
                    speedControl = True
                    direction = 'backward'
                    turn = 'left'
                    Thread_Speed_Control().start()
                    
                    #move.move(speed_set, "backward", "left", 0.8)
                    break
                if data[0] == 'STEP':
                    valOfStepAngle = int(data[1])
                    #print(data[1])
                    break
                if data[0] == 'Manipulator':
                    valOfStepAngle = 2
                    manipulatorStart = True
                    angleStepStart = True
                    
                    if data[1] == "Stop":
                        manipulatorStart = False
                        move.motorStop()
                        if cancelOnes:
                            for i in range(0, int(len(t))):
                                t[i].cancel()
                                cancelOnes = False
                        t.clear()
                        valOfStepAngle = 2
                    else:
                        manipulatorStart = True
                        
                    cmdManipulator = data[1]
                    Thread_Manipulator().start()
                    break
                if data[0] == armCmd[10]:
                    print("ARM - base")
                    manipulatorStart = False
                    manipulatorArmAngle.servoBase()
                    theta_1 = 90
                    theta_2 = 130
                    theta_3 = 130
                    theta_4 = 90
                    theta_5 = 90
                    break
                if data[0] == armCmd[11]:
                    #print("ARM - clean")
                    manipulatorStart = False
                    manipulatorArmAngle.clean_all()
                    break
                if data[0] == armCmd[12]:
                    #print("Stop motors")
                    move.motorStop()
                    break
                if data[0] == systemCmd[0]:
                    print("System shutdown now")
                    os.system("sudo shutdown now")
                    break
                if data[0] == systemCmd[1]:
                    print("System reboot")
                    os.system("sudo reboot")
                    break
                if data[0] == systemCmd[2]:
                    if len(data) == 4:
                        TakeAndWritePhoto(int(data[1]), int(data[2]), int(data[3]), "images", "my_img.png")
                        ReadByteDataFromImage()
                    else:
                        TakeAndWritePhoto(int(data[1]), int(data[2]), int(data[3]), "images", "my_img.png")
                        FindSpahesAndColors(path_to_img)
                        ReadByteDataFromImage()          
                    break
                else:
                    break
        except KeyboardInterrupt:
            print("Clear ALL")
            move.motorStop()
            move.destroy()
            manipulatorArmAngle.servoBase()
            manipulatorArmAngle.clean_all()
    move.motorStop()
    move.destroy()
    manipulatorArmAngle.servoBase()
    manipulatorArmAngle.clean_all()
    tcpSerSock.close()
    os.system("sudo systemctl stop motion.service")

'''
    @brief - leds animation 'police lights'
'''
def LedPolice():
    try:
        while policeLedStart:
            os.system("sudo python3 LED_Police_RED.py")
            print("RED")
            time.sleep(0.25);
            os.system("sudo python3 LED_Police_BLUE.py")
            print("BLUE")
            time.sleep(0.25);
    except:
        os.system("sudo python3 LED_clear.py")
        print("Turn leds off")

# COMMUNICATION CLASS - THREAD #
class Thread_Communication(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        SocketCommunication()

# SPEED OF DC MOTOR CONTROL CLASS - THREAD #
class Thread_Speed_Control(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.speedControl = speedControl
    def run(self):
        global speedControl, speed_set, current_speed, direction, turn, maxSpeed
        t = []
        cancelOnes = False

        while speedControl:
            if not maxSpeed:
                j = 0.0
                for i in range(0, int((speed_set + 10) / 10.0)):
                    t.append(Timer(j, setSpeed, [direction, turn]))
                    t[i].start()
                    j += 0.25
                maxSpeed = True
                cancelOnes = True
            time.sleep(0.05)
        if cancelOnes:
            for i in range(0, int((speed_set + 10) / 10.0)):
                move.motorStop()
                t[i].cancel()
                current_speed = 30
                cancelOnes = False
                maxSpeed = False
        t.clear()

'''
    @brief - increase value of PWM channel to control speed
             and turn on motors
    @in direction - direction of move (forward/backward)
    @in turn - turn of move (right/left)
'''
def setSpeed(direction, turn):
    global speed_set, current_speed
    if current_speed < speed_set:
        #print(current_speed)
        current_speed += 10
        move.move(current_speed, direction, turn, 0.8)

# ANIMATION LED CLASS - THREAD #
class Thread_Led_Police(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.policeLedStart = policeLedStart
    def run(self):
        LedPolice()

# MANIPULATOR ARM CONTROL CLASS - THREAD #
class Thread_Manipulator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.manipulatorStart = manipulatorStart
        self.cmdManipulator = cmdManipulator
    def run(self):
        Manipulator()
 
'''
    @brief - controling servo drivers
''' 
def Manipulator():
    global theta_2, theta_3, theta_4, theta_5, theta_camera, manipulatorStart, valOfStepAngle, speedControl, direction, turn, t, cancelOnes, angleStepStart

    Thread_Manipulator_Step_Of_Angle().start()
    
    while manipulatorStart:
        if cmdManipulator == "theta1up":
            move.move(speed_set, "no", "left", 0.8)
            time.sleep(0.05)
            move.motorStop()
        elif cmdManipulator == "theta2up":
            theta_2 = manipulatorArmAngle.servoMoveUp(int(12), theta_2, valOfStepAngle)
        elif cmdManipulator == "theta3up":
            theta_3 = manipulatorArmAngle.servoMoveUp(int(13), theta_3, valOfStepAngle)
        elif cmdManipulator == "theta4up":
            theta_4 = manipulatorArmAngle.servoMoveUp(int(14), theta_4, valOfStepAngle)
        elif cmdManipulator == "theta5up":
            theta_5 = manipulatorArmAngle.servoMoveUp(int(15), theta_5, 20)
        elif cmdManipulator == "theta6up":
            theta_camera = manipulatorArmAngle.servoMoveDownCamera(int(11), theta_camera, 2)
        elif cmdManipulator == "theta1down":
            move.move(speed_set, "no", "right", 0.8)
            time.sleep(0.05)
            move.motorStop()
        elif cmdManipulator == "theta2down":
            theta_2 = manipulatorArmAngle.servoMoveDown(int(12), theta_2, valOfStepAngle)
        elif cmdManipulator == "theta3down":
            theta_3 = manipulatorArmAngle.servoMoveDown(int(13), theta_3, valOfStepAngle)
        elif cmdManipulator == "theta4down":
            theta_4 = manipulatorArmAngle.servoMoveDown(int(14), theta_4, valOfStepAngle)
        elif cmdManipulator == "theta5down":
            theta_5 = manipulatorArmAngle.servoMoveDown(int(15), theta_5, 20)
        elif cmdManipulator == "theta6down":
            theta_camera = manipulatorArmAngle.servoMoveUpCamera(int(11), theta_camera, 2)
        elif cmdManipulator == "Stop":
            move.motorStop()

# MANIPULATOR ARM STEP VALUE OF ANGLE CLASS - THREAD #
class Thread_Manipulator_Step_Of_Angle(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.manipulatorStart = manipulatorStart
    def run(self):
        global valOfStepAngle, t, cancelOnes, angleStepStart
        t = []
        t.clear()
        cancelOnes = False
        
        while manipulatorStart:
            time.sleep(0.05)
            j = 0.0
            if angleStepStart:
                for i in range(0, 5):
                    t.append(Timer(j, setAngleStep))
                    time.sleep(0.05)
                    t[i].start()
                    j += 0.5
                    cancelOnes = True
                    angleStepStart = False
                time.sleep(0.05)

'''
    @brief - increase value of step angle to Timer inside Thread
'''
def setAngleStep():
    global valOfStepAngle, valOfStepAngle_MAX
    
    if valOfStepAngle < valOfStepAngle_MAX/2:
        valOfStepAngle += 1

# LINE TRACKING MODE CLASS - THREAD #
class Thread_Line_Tracking(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.lineTrackingStart = lineTrackingStart
    def run(self):
        global speed_set
        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(line_pin_right,GPIO.IN)
            GPIO.setup(line_pin_middle,GPIO.IN)
            GPIO.setup(line_pin_left,GPIO.IN)
            move.setup()
            counter = 0

            while lineTrackingStart:
                status_right = GPIO.input(line_pin_right)
                status_middle = GPIO.input(line_pin_middle)
                status_left = GPIO.input(line_pin_left)
                
                line_tracking_sensor_data['sensor1'] = status_left
                line_tracking_sensor_data['sensor2'] = status_middle
                line_tracking_sensor_data['sensor3'] = status_right
                line_tracking_sensor_data['counter'] = counter
                
                tmp = line_tracking_sensor_data
                tmp_json = json.dumps(tmp)
                
                # save to file
                try:
                    file = open('/home/pi/server/appServer/line_tracking_sensor.json', "w")
                    file.write(tmp_json)
                except:
                    print("Write Error - RPY")
                finally:
                    file.close()
                
                if counter >= 6:
                    counter = 0
                
                print('R%d   M%d   L%d'%(status_right, status_middle, status_left))
                if status_middle == 1:
                    if status_left == 0 and status_right == 0:
                        move.move(40, 'forward', 'no', 0.8)
                        time.sleep(0.05)
                    elif status_left == 1 and status_right == 0:
                        #print("right")
                        move.move(60, 'no', 'right', 0.8)
                        time.sleep(0.05)
                    elif status_right == 1 and status_left == 0:
                        #print("left")
                        move.move(60, 'no', 'left', 0.8)
                        time.sleep(0.05)
                    elif status_right == 1 and status_left == 1:
                        #print("stop")
                        counter += 1
                        print(counter)
                        
                        line_tracking_sensor_data['sensor1'] = status_left
                        line_tracking_sensor_data['sensor2'] = status_middle
                        line_tracking_sensor_data['sensor3'] = status_right
                        line_tracking_sensor_data['counter'] = counter
                        
                        tmp = line_tracking_sensor_data
                        tmp_json = json.dumps(tmp)
                        
                        # save to file
                        try:
                            file = open('/home/pi/server/appServer/line_tracking_sensor.json', "w")
                            file.write(tmp_json)
                        except:
                            print("Write Error - RPY")
                        finally:
                            file.close()
                            
                        move.motorStop()
                        time.sleep(2)   
                        move.move(40, 'forward', 'no', 0.8)
                        time.sleep(0.1)   
                else:
                    if status_left == 1 and status_right == 0:
                        #print("right")
                        move.move(60, 'no', 'right', 0.8)
                        time.sleep(0.05)
                    elif status_right == 1 and status_left == 0:
                        #print("left")
                        move.move(60, 'no', 'left', 0.8)
                        time.sleep(0.05)
                    elif status_right == 1 and status_left == 1:          
                        #print("stop")
                        move.move(40, 'backward', 'no', 0.8)
                        time.sleep(0.05)
                        move.motorStop()
                    elif status_right == 0 and status_left == 0:
                        #print("stop")
                        move.move(40, 'backward', 'no', 0.8)
                        time.sleep(0.05)
            move.motorStop()
        except:
            move.motorStop()

###########################################################
###############  AUTO MODE FOLLOW OBJECT  #################
###########################################################

frameWidth = 640
frameHeight = 480

threshold1 = 166
threshold2 = 171

blue_lower = np.array([94, 111, 111])
blue_upper = np.array([111, 219, 188])

red_lower = np.array([158, 84, 126])
red_upper = np.array([199, 153, 255])

green_lower = np.array([32, 120, 100])
green_upper = np.array([85, 242, 130])

yellow_lower = np.array([19, 107, 89])
yellow_upper = np.array([35, 255, 255])

deadZone = 100

global imgContour

'''
    @brief - follow object mode
    @in img -  photo for processing
    @in imgContour -  photo for results
    @return dir - dierction of move
'''
def getDirectionOfMove(img, imgContour):
    dir = "Nothing detected"
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = 1550
        areaMax = 10000
        
        if area > areaMin and area < areaMax:
            #cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            '''
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)

            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                        (0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                        (0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX,
                        0.7,
                        (0, 255, 0), 2)
            '''
            # Center values
            cx = int(x + (w / 2))
            cy = int(y + (h / 2))

            if (cx < int(frameWidth / 2) - deadZone):
                '''
                cv2.putText(imgContour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (0, int(frameHeight / 2 - deadZone)),
                              (int(frameWidth / 2) - deadZone, int(frameHeight / 2) + deadZone), (0, 0, 255),
                              cv2.FILLED)
                '''
                dir = "Left"
            elif (cx > int(frameWidth / 2) + deadZone):
                '''
                cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)),
                              (frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
                '''
                dir = "Right"
            elif (cy < int(frameHeight / 2) - deadZone):
                '''
                cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), 0),
                              (int(frameWidth / 2 + deadZone), int(frameHeight / 2) - deadZone), (0, 0, 255),
                              cv2.FILLED)
                '''
                dir = "Up"
            elif (cy > int(frameHeight / 2) + deadZone):
                '''
                cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                cv2.rectangle(imgContour, (int(frameWidth / 2 - deadZone), int(frameHeight / 2) + deadZone),
                              (int(frameWidth / 2 + deadZone), frameHeight), (0, 0, 255), cv2.FILLED)
                '''
                dir = "Down"
            else:
                dir = "Center"
            #cv2.line(imgContour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy), (0, 0, 255), 3)
    return dir

# FOLLOW OBJECT MODE CLASS - THREAD #
class Thread_Follow_Object(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.cap = cap
        self.followObjectStart = followObjectStart
        self.followColorName = followColorName
    def run(self):
        global theta_camera, theta_1, theta_2, theta_3, theta_4, theta_5
        while followObjectStart:
            _, img = cap.read()
            imgContour = img.copy()
            imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            blue_mask = cv2.inRange(imgHsv, blue_lower, blue_upper)
            red_mask = cv2.inRange(imgHsv, red_lower, red_upper)
            green_mask = cv2.inRange(imgHsv, green_lower, green_upper)
            yellow_mask = cv2.inRange(imgHsv, yellow_lower, yellow_upper)

            result = cv2.bitwise_and(img, img, mask=yellow_mask)

            if followColorName == 'Yellow':
                result = cv2.bitwise_and(img, img, mask=yellow_mask)
            if followColorName == 'Red':
                result = cv2.bitwise_and(img, img, mask=red_mask)
            if followColorName == 'Blue':
                result = cv2.bitwise_and(img, img, mask=blue_mask)
            if followColorName == 'Green':
                result = cv2.bitwise_and(img, img, mask=green_mask)
                print("green")
            imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
            imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

            imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
            kernel = np.ones((5, 5))
            imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

            direction = getDirectionOfMove(imgDil, imgContour)
            
            distance = ultrasonic.checkdist() 
            
            if direction == 'Down':
                theta_camera = manipulatorArmAngle.servoMoveUpCamera(int(11), theta_camera, 6)
            elif direction == 'Up':
                theta_camera = manipulatorArmAngle.servoMoveDownCamera(int(11), theta_camera, 6)
            elif direction == 'Left':
                move.move(50, "no", "left", 0.8)
            elif direction == 'Right':
                move.move(50, "no", "right", 0.8)
            elif direction == 'Center':
                if distance < 20.0:
                    move.move(50, "backward", "no", 0.8)
                    time.sleep(0.05)
                    move.motorStop()
                elif distance > 40.0:
                    move.move(50, "forward", "no", 0.8)
                    time.sleep(0.05)
                    move.motorStop()
                else:
                    move.motorStop()
            else:
                move.motorStop()
 
            print(direction)
            print(distance)
            
        manipulatorArmAngle.servoBase()
        theta_1 = 90
        theta_2 = 130
        theta_3 = 130
        theta_4 = 90
        theta_5 = 90
        move.motorStop()
        manipulatorArmAngle.clean_all()

###########################################################
###############  AUTO MODE SEGREGATE  #####################
###########################################################

turn_left = True
turn_right = True
enPick = True
enLose = False

counter = 0
step = 0

shape = "-"
color = "-"

status_right = 0
status_middle = 0
status_left = 0

distance_forward = 0.0

'''
    @brief - Segregate mode loop for thread
'''
def SegregateAutoMode():   
    global counter, step
    while segregateStart:
        Segregate()
    move.motorStop()
    counter = 0
    step = 0

# SEGREGATE MODE CLASS - THREAD #
class Thread_Segregate(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.segregateStart = segregateStart
    def run(self):
        while segregateStart:
            SegregateAutoMode()

'''
    @brief init and setup GPIO 
'''
def setup_Segregate():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(line_pin_right,GPIO.IN)
    GPIO.setup(line_pin_middle,GPIO.IN)
    GPIO.setup(line_pin_left,GPIO.IN)

'''
    @brief line tracking and motor control function
    @param lineTrackingStart - enable to start tracking
'''
def run():
    global step, counter, status_right, status_middle, status_left, turn_left, turn_right, shape, color, enPick, en
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    print('R%d   M%d   L%d'%(status_right, status_middle, status_left))
    #time.sleep(0.05)
    if status_middle == 1:
        if status_left == 0 and status_right == 0:
            move.move(40, 'forward', 'no', 0.8)
            time.sleep(0.05)
        elif status_left == 1 and status_right == 0:
            #print("right")
            move.move(60, 'no', 'right', 0.8)
            time.sleep(0.05)
        elif status_right == 1 and status_left == 0:
            #print("left")
            move.move(60, 'no', 'left', 0.8)
            time.sleep(0.05)
        elif status_right == 1 and status_left == 1:
            #print("stop")
            counter += 1
            #print(counter)
            move.motorStop()
            time.sleep(1)
            
            turn_left = True
            turn_right = True
    else:
        if status_left == 1 and status_right == 0:
            #print("right")
            move.move(60, 'no', 'right', 0.8)
            time.sleep(0.05)
        elif status_right == 1 and status_left == 0:
            #print("left")
            move.move(60, 'no', 'left', 0.8)
            time.sleep(0.05)
        elif status_right == 1 and status_left == 1:          
            #print("stop")
            move.move(40, 'backward', 'no', 0.8)
            time.sleep(0.05)
            move.motorStop()
        elif status_right == 0 and status_left == 0:
            #print("stop")
            move.move(40, 'backward', 'no', 0.8)
            time.sleep(0.05)

'''
    @brief - Pick item from place A
    @return shape - (string) name of found shape
    @return color - (string) name of found color
''' 
def FindAndPick():
    global step, status_right, segregateStart, status_middle, status_left, turn_left, turn_right, shape, color, distance_forward
    
    Rotation("no", "right")
    time.sleep(0.4)
    Read()
    distance_forward = ultrasonic.checkdist()
    while (status_middle == 0 or status_left == 1) and turn_right and distance_forward > 20.0:
        Read()
        Rotation("no", "right")
        Read()
        distance_forward = ultrasonic.checkdist()
        
    turn_right = False
    Rotation("no", "right")
    time.sleep(0.4)
    move.motorStop()
    
    shape = "-"
    color = "-"
    
    FindSpaheAndColorSegregate(path_to_img)
    while (shape == "-" and color == "-" ) and segregateStart: 
        FindSpaheAndColorSegregate(path_to_img)
    ReadByteDataFromImage() 
    
    manipulatorArmAngle.servoGrab()
    
    print("OpenCV")
    
    time.sleep(1)
    
    Rotation("no", "left")
    time.sleep(0.5)
    
    Read()
    while (status_middle == 0 or status_right == 1) and turn_left and not turn_right and distance_forward > 20.0:
        Read()
        Rotation("no", "left")
        Read()
        distance_forward = ultrasonic.checkdist()
    turn_left = False
    
    Rotation("no", "left")
    time.sleep(0.35)
    move.motorStop()
    
    Rotation("forward", "no")
    time.sleep(0.2)
    move.motorStop()
    
    return shape, color

'''
    @brief - Lose item into place B
'''     
def FindAndLose():
    global status_right, status_middle, status_left, turn_left, turn_right
    
    Rotation("no", "right")
    time.sleep(0.4)
    Read()
    distance_forward = ultrasonic.checkdist()
    while (status_middle == 0 or status_left == 1) and turn_right and distance_forward > 20.0:
        Read()
        Rotation("no", "right")
        Read()
        distance_forward = ultrasonic.checkdist()
    turn_right = False
    Rotation("no", "right")
    time.sleep(0.4)
    move.motorStop()

    manipulatorArmAngle.servoLose()
    
    print("Lose")
    
    time.sleep(1)
    
    Rotation("no", "left")
    time.sleep(0.5)
    
    Read()
    distance_forward = ultrasonic.checkdist()
    while (status_middle == 0 or status_right == 1) and turn_left and not turn_right and distance_forward < 20.0:
        Read()
        Rotation("no", "left")
        Read()
        distance_forward = ultrasonic.checkdist()
        
    turn_left = False
    Rotation("no", "left")
    time.sleep(0.35)
    move.motorStop()
    
    Rotation("forward", "no")
    time.sleep(0.2)
    move.motorStop()

'''
    @brief - move robot 
    @in direction1 - forward/backward
    @in direction2 - left/right
'''  
def Rotation(direction1, direction2):
    move.move(80, direction1, direction2, 0.9) # obrot o 90 stopni

'''
    @brief - Read data from line tracking sensor
'''  
def Read():
    global status_right, status_middle, status_left
    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    print('R%d   M%d   L%d'%(status_right, status_middle, status_left))

'''
    @brief - Read data from line tracking sensor for check position
    @return status_left - (1/0) staus of left sensor
    @return status_middle - (1/0) staus of middle sensor
    @return status_right - (1/0) staus of right sensor
'''  
def ReadForCheck():
    global status_right, status_middle, status_left

    status_right = GPIO.input(line_pin_right)
    status_middle = GPIO.input(line_pin_middle)
    status_left = GPIO.input(line_pin_left)
    
    return status_left, status_middle, status_right

'''
    @brief - Segregate mode for thread 
             the sequence of steps and transition conditions that the robot must perform
'''  
def Segregate():
    global enPick, enLose, counter, step, status_right, status_middle, status_left, turn_left, turn_right, shape, color
    
    if counter >= 6:
        counter = 0
    
    print("Step: "+str(step)+" Counter: "+str(counter))
    
    # Step 0 -> go to first collection point 
    if step == 0:
        run()
        if step == 0 and counter == 1:
            step = 1
    # Step 1 -> check shape and color   
    elif step == 1:
        shape, color = FindAndPick()
        print("shape: " + str(shape) +" Color: " +str(color))
        if step == 1:
            step = 2
    # Step 2 -> go to place depended on shape/color found     
    elif step == 2 and (shape == "Rectangle" or shape == "Square" or color == "Blue"):
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 2 and counter == 4:
            step = 3
    elif step == 2 and (shape == "Triangle" or color == "Green" or color == "Green"):        
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 2 and counter == 5:
            step = 3     
    elif step == 2 and (shape == "Circle" or color == "Yellow"): 
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 2 and counter == 6:
            step = 3
    # Step 3 -> stop on first lose place "Rectangle"/"Square"/"Red"
    elif step == 3:
        FindAndLose()
        if step == 3:
            step = 4
    # Step 4 -> go to second collection point 
    elif step == 4:
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 4 and counter == 2:
            step = 5
    # Step 5 -> check shape and color on second position 
    elif step == 5:
        shape, color = FindAndPick()
        print("shape: " + str(shape) +" Color: " +str(color))
        if step == 5:
            step = 6
    # Step 6 -> go to place depended on shape/color found     
    elif step == 6 and (shape == "Rectangle" or shape == "Square" or color == "Blue"):
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 6 and counter == 4:
            step = 7
    elif step == 2 and (shape == "Triangle" or color == "Green" or color == "Green"):       
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 6 and counter == 5:
            step = 7     
    elif step == 6 and (shape == "Circle" or color == "Yellow"): 
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 6 and counter == 6:
            step = 7
    # Step 7 -> stop on firt lose place "Triangle"/"Green"
    elif step == 7:
        FindAndLose()
        if step == 7:
            step = 8
    # Step 8 -> go to third collection point 
    elif step == 8:
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 8 and counter == 3:
            step = 9
    # Step 9 -> check shape and color on third position 
    elif step == 9:
        shape, color = FindAndPick()
        print("shape: " + str(shape) +" Color: " +str(color))
        if step == 9:
            step = 10
    # Step 10 -> go to place depended on shape/color found     
    elif step == 10 and (shape == "Rectangle" or shape == "Square" or color == "Blue"):
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 10 and counter == 4:
            step = 11
    elif step == 10 and (shape == "Triangle" or color == "Green" or color == "Green"):       
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 10 and counter == 5:
            step = 11     
    elif step == 10 and (shape == "Circle" or color == "Yellow"): 
        #move.move(50, 'forward', 'no', 0.8)
        #time.sleep(0.15)
        run()
        if step == 10 and counter == 6:
            step = 11
    # Step 11 -> stop on first lose place "Circle"/"Blue"
    elif step == 11:
        FindAndLose()
        if step == 11:
            step = 12  

    elif step == 12:
        print("Work ended")
        time.sleep(2)
        
### MAIN ###
if __name__ == "__main__":
    ''' COMUNICATION THREAD  '''
    Thread_Communication().start()