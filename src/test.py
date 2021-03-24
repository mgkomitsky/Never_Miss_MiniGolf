'''import numpy as np






a = np.array([2,1,0])
b = np.array(['c','b','a'])
c =np.full(3,None).tolist()




for (number,letter) in zip(a,b):
    c[number] = [number,letter]

    


print(c)


#I expected this to output [[0, 'a'], [1, 'b'], [2, 'c']]

#Why does it not?'''



list1 = []
list2 = []

for i in range(38):
    x = (974*(687**(38*i)))%1373
    y = (2**i)%1373
    list1.append(x)
    list2.append(y)

print(list1)
print(list2)

for number in list1:
    for value in list2:
        if number == value:
            print(number)
            print(list1.index(number))


'''x = 0
ans = 0

while ans != 250:
    ans = (7**(27*x))%433
    x += 1

print(x)'''