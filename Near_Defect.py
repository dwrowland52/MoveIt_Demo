#!/usr/bin/env python

# %%

# This is the idea for the listener script

# Required Imports
#import open3d as o3d
#from multiprocessing.pool import RUN
import os
import time
import numpy as np
from find_pt_cloud_Diff import find_Differences
#import sys
#import copy
import rospy
#import moveit_commander
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped

# %%

# Functions used for listener node.

# Search gets called whenever there is data in the topic "EE_XYZ"
def search(data):

    # Stores EE Point data into xyz_UR3
    xyz_UR3 = data
    Str_Time = time.time()
    print('Began for loop through def loc\n')
    # For each location inside the def_loc list
    for loc in def_loc:

        #Find the linear distance between the two points
        dist = np.linalg.norm(loc - [xyz_UR3.x, xyz_UR3.y, xyz_UR3.z])

        # If the distance is ever less than the desired threshold then we will send a twist command
        # 0,0254 is one inch in meters
        if dist < 0.0254:
            ## Close to found defect - Send twist
            twist = TwistStamped()
            twist.linear.z = 1
            pub.publish(twist)
            print('Close')
            break
        # Else we want to continue looping through the list of defect locations
        else:
            #Continue looping
            continue
        
    END_Time = time.time()
    RUN_Time = END_Time - Str_Time
    print('For Loop Run Time: ' + str(RUN_Time))

# Function declaration of the listener node
def listen():

    t = time.localtime()
    current_time = time.strftime("%H:%M:%S", t)
    print('Start Listening at: ' + current_time + '\n')

    #Spin stops node from closing until entire ros core is off?
    rospy.spin()


# %%

if __name__ == '__main__':

    #Naming our node to EE_Pose
    cwd = os.getcwd()
    # File names of PCD data ~ Assuming within same directory
    clean_PCD = cwd + '/point_test_data2_no_defects_1650316080972000.pcd'
    defect_PCD = cwd + '/point_test_data2_1650316030972999.pcd'

    # Calling find_Differences function to store calculated difference locations
    def_loc = np.asarray(find_Differences(clean_PCD, defect_PCD))

    #Starting a listener node titled UR3_EE_XYZ
    rospy.init_node('UR3_EE_XYZ', anonymous=True)
    # The created node subscribes to the EE_XYZ
    rospy.Subscriber("EE_XYZ", Point, search)
    #Declaring where our information is being published to "EE_XYZ" - Change later
    pub = rospy.Publisher('/jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=10)

    #Calling listen function
    listen()
