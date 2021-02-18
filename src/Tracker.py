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
import time
import copy

class Tracker():

    def __init__(self):
        

        
        self.targetPoints = []
        self.boundaries = [[0,100,800,100],[0,500,800,500],[800,100,800,500]]

        


        self.holeLocation = None


         



        self.reset = False
        self.ser = None
        self.cap = None
        self.currentFrame = None
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("Target Mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        self.BALL_HSV = [[0, 0, 0], [255, 255, 255]]
        self.TARGET_HSV = [[0, 0, 0], [255, 255, 255]]
        self.f = MiniGolfKalmanFilter.MiniGolfKalmanFilter(intial_state=[0,0,1,0])

    def nothing(self, x):
        pass

    def drawTrackbars(self,window):

        cv2.namedWindow(window)

        cv2.createTrackbar("Ball LH", window, 0, 255, self.nothing)
        cv2.createTrackbar("Ball LS", window, 0, 255, self.nothing)
        cv2.createTrackbar("Ball LV", window, 125, 255, self.nothing)
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
        #self.currentFrame = cv2.rotate(self.currentFrame, rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

    def calculateBall(self, mask):  

        contours = self.findContours(mask)
        self.f.predict()
        ((contour_x, contour_y), contour_radius) = self.checkStatusOfContour(contours)
        cv2.circle(self.currentFrame, (int(contour_x), int(contour_y)),int(contour_radius), (0, 255, 0), 2) #Draw ball 
        cv2.circle(self.currentFrame, (int(self.f.x[0]), int(self.f.x[1])), int(contour_radius), (0, 255, 255), 2)   #Drawing Kalman tracking ball
        self.calculateTargetPoints()
        
        

   
        

    def drawLineToTargetPoints(self):
        
        currentPoint = [self.f.x[0],self.f.x[1]]

        myPoints = copy.deepcopy(self.targetPoints)
        
        for point in myPoints:
            if type(point) == str:
                break
            nextPoint = point
            
            
                

                
                            #Draw the line to the target point
            cv2.line(       self.currentFrame, 
            (int(currentPoint[0]), int(currentPoint[1])) ,
            (int(nextPoint[0]), int(nextPoint[1])),      
            (0,255,120) ,1)
            currentPoint = point
    

    def findHole(self, mask):

        contours = self.findContours(mask)
        contour_y = 0
        if len(contours) > 0:
            contours = max(contours, key=cv2.contourArea)

            ((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)
        
        self.holeLocation = [contour_x,contour_y]
       

        
        
    def findContours(self, mask):
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        #if len(contours) > 0:
            #contours = max(contours, key=cv2.contourArea)
            #((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)
       
        return contours

    

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

    def returnAllPointsOfIntersection(self,x1,y1,x2,y2):
        
        
        points = []
        for line in self.boundaries:
    
            x3,y3,x4,y4 = line[0],line[1],line[2],line[3]
            #t
            num =  (x1 - x3)*(y3 - y4)-(y1 - y3)*(x3 - x4)
            den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)

            if den == 0:
                continue
            t = num/den
            #u
            num =  (x2 - x1)*(y1 - y3)-(y2 - y1)*(x1 - x3)
            den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
            
            u = num/den
            if den == 0:
                continue

            x = x1 + t*(x2 - x1)
            y = y1 + t*(y2 - y1)

            position = [x1,y1] 

            if self.isPointOnBoundary(position,[x3,y3],[x4,y4]):
                
                continue

            if u >= 0 and u <= 1 and t >= 0:
                points.append([x,y])

            


        return points
        



    def isPointOnBoundary(self,point,a,b):
    #We need to form 2 vectors
        x = [a[0]-point[0],a[1]-point[1]]   #Point to a
        y = [b[0]-point[0],b[1]-point[1]]    #Point to b
        c = np.cross(x,y)
        
        if c != 0:
            #print("False")
            return False
        else:
            #print("True")
            return True

    

    def returnClosest(self,points):
        
        if len(points) == 0:
            return "No Points"
        
        distances = []
        for point in points:
            #calculate the distance
            d = ((point[0]-self.f.x[0])**2+(point[1]-self.f.x[1])**2)**.5
            distances.append(d)

        
      

        indexes = []
        for x in range(len(distances)):
            indexes.append(x)
            
                
        for x in range(len(distances)):
            already_sorted = True

            for y in range(len(distances)-1):
                if distances[y] > distances[y+1]:
                    tmp = distances[y]
                    distances[y] = distances[y+1]
                    distances[y+1] = tmp

                    tmp = indexes[y]
                    indexes[y] = indexes[y+1]
                    indexes[y+1] = tmp
                    already_sorted = False

                    
            if already_sorted:
                break
        
        return points[indexes[0]]
        
            

    def norm(self,a,b):
        magnitude  = (a**2 + b**2)**.5
        x = a/magnitude 
        y = b/magnitude
        

        return [x,y]

    def calculateTargetPoints(self):

        self.targetPoints = []
        points = self.returnAllPointsOfIntersection(self.f.x[0],self.f.x[1],self.f.x[0]+self.f.x[2],self.f.x[1]+self.f.x[3])
        
        initialPoint = self.returnClosest(points)
        if type(initialPoint) != str:
            self.targetPoints.append(initialPoint)
            
            
            

        
                
        self.calculateBounce(initialPoint)
        
        self.drawLineToTargetPoints()
            

            


                
                
                
                
    def calculateBounce(self,initialPoint):

        velocity = [self.f.x[2],self.f.x[3]]
        
        
        while True:
            velocity[1] = -velocity[1]
            try:
                allPoints = self.returnAllPointsOfIntersection(initialPoint[0],initialPoint[1],initialPoint[0]+velocity[0],initialPoint[1]+velocity[1])
            except TypeError:
                return
            nextPoint = self.returnClosest(allPoints)
            if type(nextPoint) != str:
                self.targetPoints.append(nextPoint)
            initialPoint = nextPoint
            if initialPoint[0] == 800:
                break
            

    def applyMask(self, frame, lower, upper, window):  # Apply the mask
        FRAME_IN_HSV_SPACE = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(FRAME_IN_HSV_SPACE,
                           np.float32(lower), np.float32(upper))
        cv2.imshow(window, mask)
        return mask

    def calculateCommand(self):
        #print(self.targetPoints)
        index = len(self.targetPoints)
        

        if index > 0:
        
            if self.targetPoints[-1][1] > self.holeLocation[1]:
                    
                print("DOWN")
                
                    #self.sendCommandToMCU("s")
                    #self.sendCommandToMCU("l")
            elif self.targetPoints[-1][1] < self.holeLocation[1]:
                
                print("UP")
                    #self.sendCommandToMCU("s")
                    #self.sendCommandToMCU("r")
            else: 
                    
                    #self.sendCommandToMCU("s")
                print("STAY")

        else:
            print("STAY")
    
            


    def sendCommandToMCU(self,command):
        self.ser.write(str.encode(command))

    def initializeSerialPort(self):
        self.ser = serial.Serial('COM3')

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

    def autoHome(self):
        pass

    def openWindow(self, windowName, l=960, w=540):


        cv2.namedWindow(windowName)
        cv2.resizeWindow(windowName, l, w)


    def drawBoundaries(self):
        for line in self.boundaries:
            cv2.line(       self.currentFrame, 
            (int(line[0]), int(line[1])) ,
            (int(line[2]), int(line[3])),      
            (150,255,120) ,1)


    def autoSetBoundaries(self):
        pass