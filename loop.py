import cv2
import numpy as np
import time
import imutils
from filterpy.kalman import KalmanFilter
from src.Tracker import Tracker
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--file_name','-f',type=str, default=0)
args = parser.parse_args()

track = Tracker()
#if file name is provided, setupVideoStream will use the file 
#otherwise it will use 0, leading to camera capture
#track.setupVideoStream(args.file_name)
track.setupVideoStream(args.file_name)
cv2.namedWindow("Trackbars")
track.drawTrackbars("Trackbars")
#track.initializeSerialPort()




while(True):
    #track.checkESPState()
    track.returnTrackbarPosition("Trackbars")
    track.setFrame()
    
    #time.sleep(.01)            

    ball_mask = track.applyMask(track.currentFrame,
                                track.BALL_HSV[0], track.BALL_HSV[1], "Mask")

    target_mask = track.applyMask(track.currentFrame,
                                track.TARGET_HSV[0], track.TARGET_HSV[1], "Target Mask")

    track.calculateBall(ball_mask)
    track.findTarget(target_mask)
    track.drawBoundaries()
    track.showFrame()
    
    #track.calculateCommand()
    
    
    
    
    
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
track.cap.release()
cv2.destroyAllWindows()
track.closeSerialPort()


#loop.py -f data\test4.mkv 