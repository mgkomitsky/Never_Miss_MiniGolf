import cv2
import numpy as np
import time
import imutils
from filterpy.kalman import KalmanFilter
from src.Tracker import Tracker
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--file_name', '-f', type=str, default=0)
args = parser.parse_args()



track = Tracker()
track.initializeSerialPort()
track.setupVideoStream(0)
track.setFrame()








while(True):
    
    #track.checkESPState()
   
    #time.sleep(.75)
    track.setFrame()
    
    track.updateMarkers()
    track.autoSetBoundaries()
    track.blackout()
    
    track.returnTrackbarPosition("Trackbars")
    
    
    
    

    target_mask = track.applyMask(track.currentFrame, track.TARGET_HSV[0], track.TARGET_HSV[1], "Target Mask")
    track.findHole(target_mask)

    ball_mask = track.applyMask(track.currentFrame, track.BALL_HSV[0], track.BALL_HSV[1], "Mask")
    track.calculateBall(ball_mask)
    

    
   
    track.drawBoundaries()
    track.calculateCommand()
    
    track.showFrame()
 
    
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

track.sendCommandToMCU('s')
track.cap.release()
cv2.destroyAllWindows()
#track.closeSerialPort()


# loop.py -f data\test9.mkv
