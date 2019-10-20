#!/usr/bin/env python3
#!/usr/bin/env python
import numpy as np
import math

# Provided map from map.txt
map1 = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
            [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
            [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
            [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
            [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
            [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
            [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
            [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
            [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
            [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
            [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

# Converting map to array
map1 = np.asarray(map1).reshape(20,18)

#start = [11, 0]
#goal = [0, 13]

# reversing path to find the parent path
def reconstruct_path(cameFrom, current):
    total_path = [current]
    current = cameFrom[current[0]][current[1]]
    total_path.append(current)
    #print(current)
    while current in np.reshape(cameFrom, 360).tolist() and current != None:
        current = cameFrom[current[0]][current[1]]
        total_path.append(current)
    return total_path

#Finding neighours considering a point  
def neighbours(current):
    i = current[0]
    j = current[1]
    r=len(map1)
    c=len(map1[0])
    final_dict={}
    
    if not(i-1<0 or map1[i-1][j]==1):
        final_dict["T"]=[[i-1,j],1]
    if not(j-1<0 or map1[i][j-1]==1):
        final_dict["L"]=[[i,j-1],1]
    if not(j+1==c or map1[i][j+1]==1):
        final_dict["R"]=[[i,j+1],1]
    if not(i+1==r or map1[i+1][j]==1):
        final_dict["B"]=[[i+1,j],1]
    if not(i-1<0 or j-1<0 or map1[i-1][j-1]==1 or (final_dict.get("T")==None and final_dict.get("L")==None)):
        final_dict["TL"]=[[i-1,j-1],1]
    if not(i-1<0 or j+1==c or map1[i-1][j+1]==1 or (final_dict.get("T")==None and final_dict.get("R")==None)):
        final_dict["TR"]=[[i-1,j+1],1]
    if not(i+1==r or j-1<0 or map1[i+1][j-1] ==1 or (final_dict.get("B")==None and final_dict.get("L")==None)):
        final_dict["BL"]=[[i+1,j-1],1]
    if not(j+1==c or i+1==r or map1[i+1][j+1]==1 or (final_dict.get("R")==None and final_dict.get("B")==None)):
        final_dict["BR"]=[[i+1,j+1],1]
    return final_dict

# Euclidean distance formula
def dist(current,neighbour):
    distance = math.sqrt(pow(neighbour[1] - current[1], 2) + pow(neighbour[0] - current[0], 2))
    return distance

# A star algorithm
def astar(start,goal):

    # creating an open set
    open_set = []
    open_set.append(start)
    cameFrom = np.empty((20, 18), dtype=list)
    gscore=np.full((20,18),np.inf)
    fscore=np.full((20,18),np.inf)
    #key = 0
    map_dict = []

    # Created dictionaries which take values as map co-ord, g and f
    d=dist(start, goal)
    map_dict.append(start)
    gscore[start[0],start[1]]=0
    fscore[start[0],start[1]]=d      

    # Creating a closed set
    closed_set = []

    
    while len(open_set) != 0:
        
        minimum = 100000
   
        temp = 0
        for i in open_set:
            d=fscore[i[0]][i[1]]
            if(minimum>d):
                minimum=d
                temp=i

        if temp == goal:

            return reconstruct_path(cameFrom, temp)

        current = temp
            
        open_set.remove(current)
        closed_set.append(current)

        # Fetched nearest neighbours for current node
        ne_dict = neighbours(current)

        for neighbour in ne_dict.values():
            if neighbour[0] in closed_set:
                continue
                
            tent_gScore = minimum + neighbour[1] + dist(neighbour[0], goal)
            x=neighbour[0][0]
            y=neighbour[0][1]
            if tent_gScore < fscore[x][y]:
                cameFrom[x][y]=current
                gscore[x][y] = tent_gScore
                fscore[x][y] = tent_gScore + dist(neighbour[0], goal)
                 
                #Appending set with its neighbour
                if neighbour[0] not in open_set:
                    open_set.append(neighbour[0])    