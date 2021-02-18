import numpy as np

def isPointOnBoundary(point,a,b):
    #We need to form 2 vectors
    x = [a[0]-point[0],a[1]-point[1]]   #Point to a

    y = [b[0]-point[0],b[1]-point[1]]    #Point to b

    c = np.cross(x,y)
    
    if c == 0:
        return True
    else:
        return False



print(isPointOnBoundary([300,100],[0,100],[1000,100]))

myList = [1,2,3]


try:

    print(myList[-1])

except IndexError:
    print("LOL")