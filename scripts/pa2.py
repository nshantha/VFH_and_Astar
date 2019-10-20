#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import astar as astar
import numpy as np

# Region variable for storing laser data
regions = [0,0,0,0,0]

# Variables used to store co-ordinates
x=0
y=0
theta=0

x1 = 0
y1 = 0
z = 0

#Flag Variables
pose_flag = 0

#Position Variables
initial_position = Point()
current_position = Point()
next_position = Point() 
goal_pos = Point() 
path=[]

next_goal=1
output_list=[]

# Method used to go towards the next position provided by vfh
def angle_towards_goal(angle,position,distance):
    global next_goal
    difference_angle = math.radians(math.fabs(angle - position))
    print("diff_angle",difference_angle,angle,position,distance)
    
    vel_msg = Twist()
    if difference_angle > 0.09:
        print("rotate")
        vel_msg.linear.x=0.0
        vel_msg.angular.z = 0.3 if difference_angle > 0 else -0.3
    else:
        
        if(distance<0.1):
            #print("stop rotating")
            vel_msg.linear.x=0.0
            vel_msg.angular.z=0.0
            next_goal=next_goal+1
        else:
            vel_msg.linear.x=0.3
            vel_msg.angular.z=0.0
            #print("stop rotating and move")

    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

# Method used to determine the target position of openings
def move(current_position,goal_pos):
    desired_angle = math.degrees(math.atan2(goal_pos[1] - current_position.y, goal_pos[0] - current_position.x))
    difference_pos = math.sqrt(pow(goal_pos[1] - current_position.y, 2) + pow(goal_pos[0] - current_position.x, 2))
    # print("desired_angle",desired_angle)
    if(desired_angle<0):
         desired_angle=360+desired_angle
    return desired_angle,difference_pos


#Laser Data function
def laser_scan(laser_msg):
    global regions
    regions = laser_msg.ranges

# Vfh algorithm
def vfh(start,next_position):
    global regions,current_position
    bins = range(0,361,10)
    openings=[]
    
    # Getting bin information which is split up into 36
    bin_data=[]
    for i in range(len(bins)-1):
        count = 0
        for j in range(10):
            if(regions[bins[i]+j] < 3.0):
                count = count +1
        bin_data.append(count)
    
    minval = 1000000     
    prev_val = 15
    for elem in range(len(bin_data)):
        if bin_data[elem]<4:
            openings.append(elem)
    
    for i in openings:
        
        # Getting the space degrees
        space = (((i*10)+5)+((i*10)+6))/2
        cur_orient = math.degrees(current_position.z)

        if (cur_orient < 0):
            cur_orient=360+cur_orient
            
        # getting the target location
        goal_target ,goal_distance =move(current_position,next_position)
        
        target=2*math.fabs(space-goal_target)
        current=1*math.fabs(space-cur_orient)
        previous=0.8*math.fabs(space-prev_val)
        
        #calculating cost
        cost=target+current+previous
        
        # Calculating minimal angle for vfh
        if(cost<minval):
           minval=cost
           min_angle=space
        prev=space

    angle_towards_goal(min_angle,cur_orient,goal_distance)

    
# Callback function for Position of the Robot
def poseCallback(pose_message):  
    global x,y,theta,initial_position,current_position,pose_flag,path,next_goal,output_list
    

    if (pose_flag == 0):
        x = pose_message.pose.pose.position.x
        y = pose_message.pose.pose.position.y
    
        rot_q = pose_message.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion((rot_q.x,rot_q.y,rot_q.z,rot_q.w))
        initial_position.x = x
        initial_position.y = y
        initial_position.z = theta
        start=[]
        start.append(int(math.fabs(9-initial_position.y)))
        start.append(int(math.fabs(initial_position.x+8)))

        # Getting goal parameters from ros parameters
        goalx=rospy.get_param("/goalx")
        goaly=rospy.get_param("/goaly")
        print("goalx: ",goalx,"goaly: ",goaly)

        goalsy=int(math.fabs(goalx+9))
        goalsx=int(math.fabs(9-goaly))
        goal = [goalsx, goalsy]
        
        # Calling Astar
        path = astar.astar(start,goal)
        print("A-Star Path:",path)
        path.remove(None)
        path=path[::-1]
        
        #If provided path from astar
        if len(path)>0:
            out = [item for t in path for item in t]

            for x in range (len(out)):
            
                if x%2==0 :
                    out[x]=9-out[x]

                elif x%2!=0 :
                    out[x]=out[x]-8

            tupil_map=([(out[i],out[i+1]) for i in range(0,len(out),2)])

            out2 = [item for t in tupil_map for item in t]

            for x in range (len(out2)):
            
                if x%2==0 :
                    out2[x+1]=out[x]

                elif x%2!=0 :
                    out2[x-1]=out[x]

            tupil_co=([(out2[i],out2[i+1]) for i in range(0,len(out2),2)])

            # Main output list transformed for ODOM Frame
            output_list = list(tupil_co)

            #Goal positions determined for the output list
            goal_pos.x = output_list[len(output_list)-1][0]
            goal_pos.y = output_list[len(output_list)-1][1]

        pose_flag = 1

    
    if (pose_flag == 1):
        x = pose_message.pose.pose.position.x
        y = pose_message.pose.pose.position.y
    
        rot_q = pose_message.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        current_position.x = x
        current_position.y = y
        current_position.z = theta
        print(current_position)

        element=output_list[next_goal]
        print(element,next_goal)
        vfh(current_position,element)


def main():

    
    rospy.init_node('pa2')

    # Subscriber for Position of the robot
    rospy.Subscriber("base_pose_ground_truth", Odometry, poseCallback)

    # Subscriber for Laser Data
    rospy.Subscriber("base_scan", LaserScan, laser_scan)

           
    rate = rospy.Rate(10)
    while not rospy.is_shutdown:


        rate.sleep()
    rospy.spin()

    

if __name__ == '__main__':
    main()
