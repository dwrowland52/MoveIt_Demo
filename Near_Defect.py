#!/usr/bin/env python

# %%

# This is the idea for the listener script

# Required Imports
#import open3d as o3d
#from multiprocessing.pool import RUN
import os
import time
import numpy as np
#from find_pt_cloud_Diff import find_Differences
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

# Functions needed to find the differences
def find_Differences(file_name_Clean_PCD, file_name_Defect_PCD):
    #Assuming Files are in the working directory

    # Imports
    import open3d as o3d
    import numpy as np
    from pytransform3d import rotations as pr
    from pytransform3d import transformations as pt
    from pytransform3d.transform_manager import TransformManager

    # Determining the known transformation matrix
    of2df = pt.transform_from_pq(np.array([-0.1, 0, 0.02, 0, 0, 0,0])) #Optical Frame to Depth Frame
    bl2df = pt.transform_from_pq(np.array([0.126918, -0.467213, 0.624013, -0.0145864, 0.721838,-0.691879, -0.00635957])) #Depth Frame to Base Link

    # Using TransformManager for easy to rotation matrices
    tm = TransformManager()
    tm.add_transform("optical_frame", "depth_frame", of2df)
    tm.add_transform("base_link", "depth_frame", bl2df)

    # Obtaining the required transformation matrix
    of2bl = tm.get_transform("optical_frame", "base_link")
    Tf= np.asarray(of2bl)

    #Create PCD Objs
    pcd_defect = o3d.io.read_point_cloud(file_name_Defect_PCD)   # reading point cloud file
    pcd_clean = o3d.io.read_point_cloud(file_name_Clean_PCD)    # reading point cloud file

    pcddown_clean = pcd_clean.voxel_down_sample(voxel_size = 0.004)
    pcddown_clean.paint_uniform_color([1.0, 0.0, 0.0])
    pcddown_defect = pcd_defect.voxel_down_sample(voxel_size = 0.004)
    pcddown_defect.paint_uniform_color([0.0, 0.0, 1.0])

    #Vis the downsized pcd data
    #o3d.visualization.draw_geometries([pcddown_clean, pcddown_defect], point_show_normal =False)

    # Cleaning input data

    # Gatering PCD data points as numpy arrays
    clean_pts = np.asarray(pcd_clean.points)
    defect_pts = np.asarray(pcd_defect.points)

    # Replacing all nan values with 0 using nan_to_num()
    clean_pts = np.nan_to_num(clean_pts, nan=0)
    defect_pts = np.nan_to_num(defect_pts, nan=0)

    #Filling the points attribute of the empty o3d obj with the cleaned data
    pcd_clean.points = o3d.utility.Vector3dVector(clean_pts)
    pcd_defect.points = o3d.utility.Vector3dVector(defect_pts)

    #Compare point clouds
    pcd_Compare = pcd_defect.compute_point_cloud_distance(pcd_clean)
    compare_pts = np.asarray(pcd_Compare)
    ind = np.where(compare_pts > 0.001)[0]
    pcd_Compare = pcd_defect.select_by_index(ind)

    #Visual compared data
    #o3d.visualization.draw_geometries([pcd_Compare])

    #Segment planes
    plane_model, inliers = pcd_Compare.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    inlier_cloud = pcd_Compare.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    defect_cloud = pcd_Compare.select_by_index(inliers, invert=True)
    #o3d.visualization.draw_geometries([defect_cloud])

    #Futher isolating work piece (wp)
    # Sphere center and radius
    center = np.array([-0.011, -0.01, 0.000])
    radius = 0.6

    points = np.asarray(defect_cloud.points)

    # Calculate distances to center, set new points
    distances = np.linalg.norm(points - center, axis=1)
    defect_cloud.points = o3d.utility.Vector3dVector(points[distances <= radius])

    wp_defects = defect_cloud.points

    defect_loc = np.asarray(wp_defects)

    #o3d.visualization.draw_geometries([defect_cloud], point_show_normal= False)

    #multiplying point cloud data with transformation matrix
    point =[]
    for i in range(len(defect_loc)):
        po = np.asarray(np.insert(defect_loc[i],3,1))
    
        prs = np.reshape(po,(4,1))
    
        p_tf = np.matmul(Tf,prs)
    
    
        p_tfrs = np.delete(p_tf,3)

        point.append(p_tfrs)

    # output
    defect_loc_BaseLink_frame=np.asarray(point) 

    return defect_loc_BaseLink_frame
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
