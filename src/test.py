import numpy as np






a = np.array([2,1,0])
b = np.array(['c','b','a'])
c =np.full(3,None).tolist()




for (number,letter) in zip(a,b):
    c[number] = [number,letter]

    


print(c)


#I expected this to output [[0, 'a'], [1, 'b'], [2, 'c']]

#Why does it not?



