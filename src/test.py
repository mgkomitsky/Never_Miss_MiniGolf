distances = [3,7,3,8,5]

for distance in distances:
            
                
        for x in range(len(distances)):
            already_sorted = True

            for y in range(len(distances)-1):
                if distances[y] > distances[y+1]:
                    tmp = distances[y]
                    distances[y] = distances[y+1]
                    distances[y+1] = tmp
                    already_sorted = False

        if already_sorted:
            break
print(distances)
