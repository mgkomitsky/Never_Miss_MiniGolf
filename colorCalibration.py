import cv2
from src.Tracker import Tracker
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--file_name', '-f', type=str, default=0)
args = parser.parse_args()


tracker = Tracker()
tracker.setupVideoStream(args.file_name)
cv2.namedWindow("Trackbars",cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
tracker.drawTrackbars("Trackbars")

while(True):
    tracker.setFrame()
    tracker.returnTrackbarPosition("Trackbars")
    

    ball_mask = tracker.applyMask(tracker.currentFrame, tracker.BALL_HSV[0], tracker.BALL_HSV[1], "Mask")
    target_mask = tracker.applyMask(tracker.currentFrame, tracker.TARGET_HSV[0], tracker.TARGET_HSV[1], "Target Mask")
    tracker.showFrame()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


tracker.cap.release()
cv2.destroyAllWindows()
