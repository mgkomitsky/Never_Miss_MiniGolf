import cv2
import cv2.aruco as aruco
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
import math 



class Tracker():

    def __init__(self):
        self.previousDirection = 0
        self.currentDirection = True
        self.previousState = []
        self.markerData = np.full(3,None).tolist()

        self.targetRailX = 800
        self.topRailY = 100
        self.bottomRailY = 400

        self.boundaries = [[0, self.topRailY, self.targetRailX, self.topRailY], [0, self.bottomRailY, self.targetRailX, self.bottomRailY], 
        [self.targetRailX, self.topRailY, self.targetRailX, self.bottomRailY]]

        self.targetPoints = []
        self.holeLocation = []

        self.reset = False
        self.ser = None
        self.cap = None
        self.currentFrame = None
        cv2.namedWindow("Trackbars",cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.namedWindow("Target Mask", cv2.WINDOW_NORMAL |cv2.WINDOW_KEEPRATIO)
        self.drawTrackbars("Trackbars")
        self.BALL_HSV = [[0, 0, 0], [255, 255, 255]]
        self.TARGET_HSV = [[0, 0, 0], [255, 255, 255]]
        self.f = MiniGolfKalmanFilter.MiniGolfKalmanFilter(
            intial_state=[0, 0, 1, 0])

        self.assist = True
        self.currentSpeed = "e"

    def nothing(self, x):
        pass

    def drawTrackbars(self, window):


        cv2.namedWindow(window)

        cv2.createTrackbar("Ball LH", window, 0, 255, self.nothing)
        cv2.createTrackbar("Ball LS", window, 127, 255, self.nothing)
        cv2.createTrackbar("Ball LV", window, 202, 255, self.nothing)
        cv2.createTrackbar("Ball UH", window, 51, 255, self.nothing)
        cv2.createTrackbar("Ball US", window, 238, 255, self.nothing)
        cv2.createTrackbar("Ball UV", window, 255, 255, self.nothing)

        cv2.createTrackbar("Target LH", window, 54, 255, self.nothing)
        cv2.createTrackbar("Target LS", window, 71, 255, self.nothing)
        cv2.createTrackbar("Target LV", window, 165, 255, self.nothing)
        cv2.createTrackbar("Target UH", window, 229, 255, self.nothing)
        cv2.createTrackbar("Target US", window, 148, 255, self.nothing)
        cv2.createTrackbar("Target UV", window, 255, 255, self.nothing)

    def returnTrackbarPosition(self, window):
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

    def setupVideoStream(self, file_name=2):
        self.cap = cv2.VideoCapture(file_name)

    def showFrame(self):
        cv2.imshow("Video", self.currentFrame)

    def setFrame(self):
        
        ret, self.currentFrame = self.cap.read()

        #self.currentFrame = cv2.rotate(self.currentFrame, rotateCode = cv2.ROTATE_90_COUNTERCLOCKWISE)

    def calculateBall(self, mask):
        self.previousState = self.f.x
        contours = self.findContours(mask)
        self.f.predict()
        ((contour_x, contour_y), contour_radius) = self.checkStatusOfContour(contours)
        cv2.circle(self.currentFrame, (int(contour_x), int(contour_y)),
                   int(contour_radius), (0, 255, 0), 2)  # Draw ball
        cv2.circle(self.currentFrame, (int(self.f.x[0]), int(self.f.x[1])), int(
            contour_radius), (0, 255, 255), 2)  # Drawing Kalman tracking ball

        if self.f.x[2] > 1 or self.f.x[3] > 1:    
            self.calculateTargetPoints()

    def drawLineToTargetPoints(self):

        currentPoint = [self.f.x[0], self.f.x[1]]
        # print(self.targetPoints)
        for point in self.targetPoints:
            if type(point) == str:
                break
            nextPoint = point

            # Draw the line to the target point
            cv2.line(self.currentFrame,
                     (int(currentPoint[0]), int(currentPoint[1])),
                     (int(nextPoint[0]), int(nextPoint[1])),
                     (0, 255, 120), 1)
            currentPoint = point

    def findHole(self, mask):

        contours = self.findContours(mask)
        contour_y = 0
        if len(contours) > 0:
            contours = max(contours, key=cv2.contourArea)

            ((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)
            self.holeLocation = [contour_x, contour_y]

    def findContours(self, mask):
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        # if len(contours) > 0:
        #contours = max(contours, key=cv2.contourArea)
        #((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contours)

        return contours

    def resetFilter(self):
        self.f = MiniGolfKalmanFilter.MiniGolfKalmanFilter()
        self.reset = True

    def checkStatusOfContour(self, contours):

        if len(contours) > 0:
            contour = max(contours,  key=cv2.contourArea)
            ((contour_x, contour_y), contour_radius) = cv2.minEnclosingCircle(contour)

            """if contour_radius < 2:
                self.resetFilter()"""
            
            self.f.update([contour_x, contour_y])
        

            if (self.reset == True):
                self.f.x = [contour_x, contour_y, 1, 0]
                self.reset = False

        else:
            (contour_x, contour_y), contour_radius = (0, 0), 10

            self.resetFilter()

        return ((contour_x, contour_y), contour_radius)

    def calculateSlope(self):
        return (self.f.x[3]/self.f.x[2])

    def returnAllPointsOfIntersection(self, x1, y1, x2, y2):

        points = []
        for line in self.boundaries:

            x3, y3, x4, y4 = line[0], line[1], line[2], line[3]
            # t
            num = (x1 - x3)*(y3 - y4)-(y1 - y3)*(x3 - x4)
            den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)

            if den == 0:
                continue
            t = num/den
            # u
            num = (x2 - x1)*(y1 - y3)-(y2 - y1)*(x1 - x3)
            den = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)

            u = num/den
            if den == 0:
                continue

            x = x1 + t*(x2 - x1)
            y = y1 + t*(y2 - y1)

            position = [x1, y1]

            if self.isPointOnBoundary(position, [x3, y3], [x4, y4]) == False:
                if u >= 0 and u <= 1 and t >= 0:
                    points.append([x, y])

        return points

    def isPointOnBoundary(self, point, a, b):
        # We need to form 2 vectors
        x = [int(a[0]-point[0]), int(a[1]-point[1])]  # Point to a
        y = [int(b[0]-point[0]), int(b[1]-point[1])]  # Point to b
        c = np.cross(x, y)

        if c != 0:
           # print("False")
            return False
        else:
            # print("True")
            return True

    def returnClosest(self, points):

        if len(points) == 0:
            return "No Points"

        distances = []
        for point in points:
            # calculate the distance
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

        # print(points[indexes[0]])
        return points[indexes[0]]

    def calculateTargetPoints(self):

        self.targetPoints = []
        points = self.returnAllPointsOfIntersection(
            self.f.x[0], self.f.x[1], self.f.x[0]+self.f.x[2], self.f.x[1]+self.f.x[3])

        initialPoint = self.returnClosest(points)
        if type(initialPoint) != str:
            self.targetPoints.append(
                [int(initialPoint[0]), int(initialPoint[1])])

        #self.calculateBounce(initialPoint)

        self.drawLineToTargetPoints()

    def calculateBounce(self, initialPoint):

        velocity = [self.f.x[2], self.f.x[3]]

        while True:
            velocity[1] = -velocity[1]
            
            try:
                allPoints = self.returnAllPointsOfIntersection(
                    initialPoint[0], initialPoint[1], initialPoint[0]+velocity[0], initialPoint[1]+velocity[1])
            except TypeError:

                return
            nextPoint = self.returnClosest(allPoints)
            if type(nextPoint) != str:
                self.targetPoints.append(
                    [int(nextPoint[0]), int(nextPoint[1])])
            initialPoint = nextPoint
            
            if initialPoint[0] == self.targetRailX:
                break

    def flipVelocity(self):
        
      
        if self.currentDirection == False:
                
            print("FLIP")
            #self.f.x[3] = -self.f.x[3]
            #self.previousDirection = self.currentDirection
        self.currentDirection = True
        
    def applyMask(self, frame, lower, upper, window):  # Apply the mask
        FRAME_IN_HSV_SPACE = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(FRAME_IN_HSV_SPACE,
                           np.float32(lower), np.float32(upper))
        cv2.imshow(window, mask)
        return mask

    def calculateCommand(self):
        # print(self.targetPoints)
        index = len(self.targetPoints)

        if index > 0:

            if self.targetPoints[-1][1] >= self.holeLocation[1] + 2 or self.targetPoints[-1][1] <= self.holeLocation[1] - 2 :

                if self.f.x[2] < 1:
                    if self.assist:
                        self.sendCommandToMCU(self.currentSpeed)
                        self.sendCommandToMCU("s")
                        print("s")
                    return

                if self.targetPoints[-1][1] > self.holeLocation[1]:

                    print("r")

                    
                    if self.assist:
                        if self.holeLocation[1] >= self.bottomRailY-40 or self.holeLocation[1] <= self.topRailY+40:
                            self.sendCommandToMCU("b")
                            print("1")
                        else:
                            


                            self.sendCommandToMCU(self.currentSpeed)
                        self.sendCommandToMCU("r")



                elif self.targetPoints[-1][1] < self.holeLocation[1]:

                    print("l")
                    
                    if self.assist:
                        if self.holeLocation[1] >= self.bottomRailY-40 or self.holeLocation[1] <= self.topRailY+40:
                            self.sendCommandToMCU("b")
                            print("2")



                        else:
                            self.sendCommandToMCU(self.currentSpeed)
                        self.sendCommandToMCU("l")
            else: 
                self.sendCommandToMCU("s")
        else:
            self.sendCommandToMCU("s")

    def sendCommandToMCU(self, command):
        self.ser.write(str.encode(command))

    def initializeSerialPort(self):
        self.ser = serial.Serial('COM4')

    def closeSerialPort():
        self.ser.close

    def checkESPState(self):
        # Get json data to set states of the machine
        response = requests.get("http://52.119.101.179:7980/metrics")

        
 
      

        mode = response.json()["mode"]

        #print(mode)

        if mode == "2":
            self.assist = False
            print(self.assist)
            print("NO ASSIST")
            

        if mode == "3":
            self.assist = True
            print(self.assist)
            print("SLOW")
            self.sendCommandToMCU("b")
            self.currentSpeed = "b"

        if mode == "4":
            self.assist = True
            print(self.assist)
            print("FAST")
            self.sendCommandToMCU("e")
            self.currentSpeed = "e"

        

       



        #self.sendCommandToMCU(speed)

    def autoHome(self):
        pass

    def openWindow(self, windowName, l=540, w=540):

        cv2.namedWindow(windowName,cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(windowName, l, w)

    def drawBoundaries(self):
        for line in self.boundaries:
            cv2.line(self.currentFrame,
                     (int(line[0]), int(line[1])),
                     (int(line[2]), int(line[3])),
                     (150, 255, 120), 1)


        cv2.line(self.currentFrame,(0,self.topRailY+40),(self.targetRailX,self.topRailY+40), (255, 255,0), 1)
        cv2.line(self.currentFrame,(0,self.bottomRailY-40),(self.targetRailX,self.bottomRailY-40), (255, 255,0), 1)

    def autoSetBoundaries(self):
        self.updateMarkers()
                
                
                
        #[which marker][corners or id][which corner][x or y]          

        '''try:
            self.targetRailX = self.markerData[0][1][0][0]
            self.topRailY    = self.markerData[0][1][0][1]
            self.bottomRailY = self.markerData[1][1][1][1]
        except:
            self.targetRailX = 500
            self.topRailY =    50
            self.bottomRailY = 400'''


        self.targetRailX = 323
        self.topRailY    = 50
        self.bottomRailY = 455

        self.boundaries = [[0, self.topRailY, self.targetRailX, self.topRailY], [0, self.bottomRailY, self.targetRailX, self.bottomRailY], 
        [self.targetRailX, self.topRailY, self.targetRailX, self.bottomRailY]]

        self.boundaries = [[0, 50, 323, 50], [0, 455, 323, 455], 
        [323, 50, 323, 455]]

    def printMarker(self):
        # Load the predefined dictionary
        dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # Generate the marker
        markerImage = np.zeros((200, 200), dtype=np.uint8)
        markerImage = aruco.drawMarker(dictionary, 2, 200, markerImage, 1)

        cv2.imwrite("2.png", markerImage)

    def updateMarkers(self):
        
        
        dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # Initialize the detector parameters using defau lt values
        parameters = aruco.DetectorParameters_create()

        # Detect the markers in the image
        corners, ids, rejected = aruco.detectMarkers(self.currentFrame, dictionary, parameters=parameters)

    
        if len(corners) > 0:

            ids = ids.flatten()
            #print(ids)

            for (corner,markerid) in zip(corners,ids):
                corners = corner.reshape((4,2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                self.markerData[markerid] = [markerid,corners]


        elif len(corners) < 2:

            self.markerData = np.full(3,None).tolist()
                             
    def checkIfCloseToSwitch(self):
        if self.holeLocation[1] <= self.bottomRailY+100:
            self.sendCommandToMCU("b")
            self.currentSpeed = 'b'
        if self.holeLocation[1] >= self.topRailY-100:
            self.sendCommandToMCU("b")
            self.currentSpeed = 'b'
        
    def blackout(self):
        cv2.rectangle(self.currentFrame,(0,0),(self.targetRailX, self.topRailY),(0,0,0),-1)
        cv2.rectangle(self.currentFrame,(self.targetRailX,0),(900, 900),(0,0,0),-1)
        cv2.rectangle(self.currentFrame,(0,self.bottomRailY),(900, 900),(0,0,0),-1)
