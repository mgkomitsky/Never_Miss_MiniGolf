import cv2
import numpy as np
import imutils
from filterpy.kalman import KalmanFilter
import requests
import json
import serial
from filterpy.common import Q_discrete_white_noise
from src import MiniGolfKalmanFilter
import random

class Tracker():

    def __init__(self):

        self.targetRail_x = 1500
        self.topOfTargetRail_y = 0
        self.bottomOfTargetRail_y = 2000

        self.targetPoint = [0,0]


        self.targetPositions = [0,0]

        self.reset = False

        self.ser = None
        self.cap = None
        self.currentFrame = None
        
        
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

        cv2.namedWindow("Target Mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
       
       
        # These variables are for the mask
        
        self.BALL_HSV = [[0, 0, 0], [255, 255, 255]]
        self.TARGET_HSV = [[0, 0, 0], [255, 255, 255]]
    
        
        self.f = MiniGolfKalmanFilter.MiniGolfKalmanFilter(intial_state=[0,0,1,0])

    def nothing(self, x):
        pass

    def drawTrackbars(self,window):

        cv2.namedWindow(window)

        cv2.createTrackbar("Ball LH", window, 0, 255, self.nothing)
        cv2.createTrackbar("Ball LS", window, 0, 255, self.nothing)
        cv2.createTrackbar("Ball LV", window, 102, 255, self.nothing)
        cv2.createTrackbar("Ball UH", window, 255, 255, self.nothing)
        cv2.createTrackbar("Ball US", window, 172, 255, self.nothing)
        cv2.createTrackbar("Ball UV", window, 255, 255, self.nothing)

        cv2.createTrackbar("Target LH", window, 0, 255, self.nothing)
        cv2.createTrackbar("Target LS", window, 233, 255, self.nothing)
        cv2.createTrackbar("Target LV", window, 131, 255, self.nothing)
        cv2.createTrackbar("Target UH", window,
                           255, 255, self.nothing)
        cv2.createTrackbar("Target US", window,
                           255, 255, self.nothing)
        cv2.createTrackbar("Target UV", window,
                           255, 255, self.nothing)

    def returnTrackbarPosition(self,window):
        ball_l_h = cv2.getTrackbarPos("Ball LH", window)
        ball_l_s = cv2.getTrackbarPos("Ball LS", window)
        ball_l_v = cv2.getTrackbarPos("Ball LV", window)
        ball_u_h = cv2.getTrackbarPos("Ball UH", window)
        ball_u_s = cv2.getTrackbarPos("Ball US", window)
        ball_u_v = cv2.getTrackbarPos("Ball UV", window)

        target_l_h = cv2.getTrackbarPos("Target LH", window)
        target_l_s = cv2.getTrackbarPos("Target LS", window)
        target_l_v = cv2.getTrackbarPos("Target LV", window)
        target_u_h = cv2.getTrackbarPos("Target UH", window)
        target_u_s = cv2.getTrackbarPos("Target US", window)
        target_u_v = cv2.getTrackbarPos("Target UV", window)

        self.BALL_HSV = [[ball_l_h, ball_l_s, ball_l_v],
                         [ball_u_h, ball_u_s, ball_u_v]]

        self.TARGET_HSV = [[target_l_h, target_l_s, target_l_v],
                           [target_u_h, target_u_s, target_u_v]]

    def setupVideoStream(self, file_name=0):
        self.cap = cv2.VideoCapture(file_name)

    def showFrame(self):
        cv2.imshow("Video", self.currentFrame)

    def setFrame(self):
        ret, self.currentFrame = self.cap.read()

    def calculateBall(self, mask):  

        contours = self.findContours(mask)
        self.f.predict()
        ((contour_x, contour_y), contour_radius) = self.checkStatusOfContour(contours)
        cv2.circle(self.currentFrame, (int(contour_x), int(contour_y)),int(contour_radius), (0, 255, 0), 2) #Draw ball 
        cv2.circle(self.currentFrame, (int(self.f.x[0]), int(self.f.x[1])), int(contour_radius), (0, 255, 255), 2)   #Drawing Kalman tracking ball
        self.calculateTargetPoint()
        self.drawLineToTargetPoint()
        

    def drawLineToTargetPoint(self):
        x,y = self.targetPoint


        if(self.f.x[0] < self.targetRail_x):              #Draw the line to the target point
            cv2.line(       self.currentFrame, 
            (int(self.f.x[0]), int(self.f.x[1])) ,
            (int(x), int(y)),      
            (0,255,120) ,2)


    def findTarget(self, mask):

        contours = self.findContours(mask)
        contour_y = 0
        if len(contours) > 0:
            contours = max(contours, key=cv2.contourArea)

            ((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)
       
        self.targetPositions[1] = contour_y
        self.targetPositions[0] = self.targetPoint[1]
        return contour_y
        
    def findContours(self, mask):
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        #if len(contours) > 0:
            #contours = max(contours, key=cv2.contourArea)
            #((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)
       
        return contours

    def bounceCalculator(self):
        #If the target is not on the rail (x coordinate) , then it needs to calculate where it's going to go.
        #set x to the current target point
        #flip the slope
        #calculate next target point
        #Change target point if necessary

        
        
        pass

    def resetFilter(self):
        self.f = MiniGolfKalmanFilter.MiniGolfKalmanFilter()
        self.reset = True

    def checkStatusOfContour(self,contours):
        

        if len(contours) > 0:
            contour = max(contours,  key=cv2.contourArea)
            ((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contour)

            if contour_radius < 10:
                self.resetFilter()
            self.f.update([contour_x,contour_y])
            
            if (self.reset == True):
                self.f.x = [contour_x,contour_y,100,0]
                self.reset = False
                
        else:
            (contour_x, contour_y), contour_radius = (0,0), 10
            
            self.resetFilter()

        return ((contour_x, contour_y), contour_radius)

    def calculateSlope(self):
        return (self.f.x[3]/self.f.x[2])

    def calculateTargetPoint(self):

        x = self.targetRail_x
        m = self.calculateSlope()
        y = m*(x-self.f.x[0])+self.f.x[1]
        

        if y >= 0: #if its on the target rail
            self.targetPoint = [x,y]
            
        elif y < self.topOfTargetRail: #above target rail
            x  = (y - self.f.x[1]+self.f.x[0]*m)/m
            self.targetPoint = [x,self.topOfTargetRail_y]
            

        elif y > self.bottomOfTargetRail: #Below target rail
            x  = (y - self.f.x[1]+self.f.x[0]*m)/m
            self.targetPoint = [x,self.bottomOfBottomRail_y]
          
    
    def applyMask(self, frame, lower, upper, window):  # Apply the mask
        FRAME_IN_HSV_SPACE = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(FRAME_IN_HSV_SPACE,
                           np.float32(lower), np.float32(upper))
        cv2.imshow(window, mask)
        return mask

    def calculateCommand(self):  

        if self.targetPositions[0] > self.targetPositions[1]:
            print("DOWN")
        elif self.targetPositions[0] < self.targetPositions[1]:
            print("UP")
        else: 
            print("STAY")


    def sendCommandToMCU(self,command):
        self.ser.write(command)

    def initializeSerialPort(self):
        self.ser = serial.Serial('COM5')

    def closeSerialPort():
        self.ser.close

    def checkESPState(self):
        #Get json data to set states of the machine
        #response = requests.get("http://52.119.101.179:7980/metrics")
        text = {
	    "ip": "192.168.100.76",
	    "time": "630257",
	    "ssid": "ECEN403-Development",
	    "speed": "0",
	    "mode": "0"}

        response = json.dumps(text)

        print(response)



    def openWindow(self, windowName, l=960, w=540):
        cv2.namedWindow(windowName)
        cv2.resizeWindow(windowName, l, w)