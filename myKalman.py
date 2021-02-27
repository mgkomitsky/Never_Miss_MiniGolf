'''def calculateKalmanGain(estimateError, measurementError):
    return estimateError/(estimateError + measurementError)

def calculateCurrentEstimate(previousEstimate, kalmanGain, measurement):
    return previousEstimate + kalmanGain*(measurement-previousEstimate)

def calculateEstimateError(kalmanGain, previousEstimateError):
    return (1-kalmanGain)*(previousEstimateError)





initialEstimate = 68
previousEstimate = initialEstimate

initialEstimateError = 2
previousEstimateError = initialEstimateError
 
measurements = [75,71,70,74]
measurementError = 4

for measurement in measurements:
    KG = calculateKalmanGain(previousEstimateError,measurementError)
    currentEstimate = calculateCurrentEstimate(previousEstimate, KG, measurement)
    previousEstimate = currentEstimate
    newEstimateError = calculateEstimateError(KG,previousEstimateError)
    previousEstimateError = newEstimateError
    print(currentEstimate)'''


import numpy as np

X = np.array([1,2])
Y = np.array([[1],[2]])



dt = 1

x = 0
y = 0
dx = .8
dy = .8

stateMatrix = np.array([[x],[y],[dx],[dy]])

A = np.array([[1,0,dt,0],
    [0,1,0,dt],
    [0,0,1,0],
    [0,0,0,1]])

#print(stateMatrix,A)
print(np.dot(A,stateMatrix))









class KalmanFilter():
    def __init__(self):
        self.stateMatrix = [0,0,0,0]
        self.P = 

        