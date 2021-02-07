import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

#in this class, we inherit from the KalmanFilter class and make a specific version for our use case
class MiniGolfKalmanFilter(KalmanFilter):
    '''
    MiniGolfKalmanFilter is a wrapper of the KalmanFilter class. It assumes 2 dimenions and standard netonian physics.
    '''
    def __init__(self,
        intial_state=np.array([0.,0.,0.,0.]),
        fc = 1.,
        dt = 1.,
        R_val = 5,
        Q_val = 0.13,
        y_walls = [-10,10]
        ):
        #the super funciton runs the constructor of the KalmanFilter class and inherits it's methods for our class 
        super().__init__(dim_x=4, dim_z=2)
        #set up the Kalman filter
        self.y_walls       = y_walls
        self.intial_state  = intial_state
        self.fc            = fc
        self.dt            = 1
        self.P            *= 1000.
        self.R            *= R_val
        self.Q             = Q_discrete_white_noise(dim=4, dt=dt, var=Q_val)
        self.x             = intial_state
        self.F             = np.array([
                                      [1,    0,     dt,   0],
                                      [0,    1,     0,    dt],
                                      [0,    0,     fc,   0],
                                      [0,    0,     0,    fc]
                                      ])
        self.H             = np.array([
                                      [1.,    0.,   0.,  0.], 
                                      [0.,    1.,   0.,  0.],
                                      ])

    def print_state(self):
        print("\tcurrent possition: ({:.2f},{:.2f}) velocity: ({:.2f},{:.2f})".format(*self.x))

    def flip_y_about(self,y_val):
        '''This flips the y value around the set target y_val, and flips the velocity'''
        x,y,dx,dy = self.x
        y = 2*y_val - y
        dy = -1*dy
        self.x = np.array([x,y,dx,dy])

    def predict_and_bounce(self):
        '''mockup of how the bouncing might work'''
        self.predict()
        x,y,dx,dy = self.x
        bottom_wall, top_wall = sorted(self.y_walls)
        if y > top_wall:
            self.flip_y_about(top_wall)
        if y < bottom_wall:
            self.flip_y_about(bottom_wall)

if __name__=="__main__":    
    print("testing kalman filter wrapper")
    #define some constants
    x,y,dx,dy = 0,0,.5,1
    intial_state = np.asarray([x,y,dx,dy])

    ##
    ## Inital state:
    ##

    #create our object
    filter_eg = MiniGolfKalmanFilter(intial_state=intial_state)
    filter_eg.print_state()

    #initial_state check
    assert (filter_eg.x==intial_state).all(), "inital state wrongly initialized"

    ##
    ## After one step:
    ##

    #update once
    filter_eg.predict()
    print("after one step:")
    filter_eg.print_state()

    #predict check
    assert (filter_eg.x==np.asarray([.5,1,.5,1])).all(), "After one step, needs to be one distance down the line"

    ##
    ## After update:
    ##

    #update once
    filter_eg.update([-7,-7])
    filter_eg.predict()
    print("after updating with -7,-7")
    filter_eg.print_state()

    ##
    ## Test bouncing:
    ##
    
    print("Now, we test our bounce function. The ball willhit the default wall at -10 possition and bounce off of it")
    filter_eg.predict_and_bounce()
    filter_eg.print_state()