# %% Imports
import numpy as np
import open3d as o3d  
import matplotlib.pyplot as plt


# %% Vis original data

# Reads point cloud data
pcd = o3d.io.read_point_cloud("point_test_data2_1650316030972999.pcd")   # reading point cloud file
#print(pcd) #printing the dataset 
o3d.visualization.draw_geometries([pcd])

# %% Function to display seperated clouds
def display_inlier_outlier(cloud, ind):
    # cloud is the point cloud object in question
    # ind is the index list from the statistical seperator function

    #Statistically similar are gray
    inlier_cloud =cloud.select_by_index(ind)
    #Statistically outliers are red
    outlier_cloud = cloud.select_by_index(ind, invert =True)

    print("showing outliers(red) and inliers(gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.0, 0.0, 0.0])

    #Visualizing the seperated clouds
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    return inlier_cloud


# %% Visualized cleaned data - replaced nan with 0

#Turns pcd into a numpy array
pcd_array = np.asarray(pcd.points)
#Replacing all nan values with 0 using nan_to_num()
pcd_array = np.nan_to_num(pcd_array, nan=0)
#Creating empty o3d object
pcd_cleaned = o3d.geometry.PointCloud()
#Filling the points attribute of the empty o3d obj with the cleaned data
pcd_cleaned.points = o3d.utility.Vector3dVector(pcd_array)
# #print(type(a))
#Visualizing the cleaned data
o3d.visualization.draw_geometries([pcd_cleaned])


# %% Apply voxel to reduce # of points 

#voxel downsampling (In 3D computer graphics, a voxel represents a value on a regular grid in three-dimensional space)
print("Downsample the point cloud with a voxel of 0.004")
#Using 'pcd' and downsampling it to make it more managable
downpcd_cleaned = pcd_cleaned.voxel_down_sample(voxel_size = 0.004)
#Voxel Normal Estimation : Recompute the normal of the point cloud
downpcd_cleaned.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn =30))
#Vis the downsized pcd data
o3d.visualization.draw_geometries([downpcd_cleaned], point_show_normal =False)


# %% Breaks pointcloud into seperate planes

# Seperating cleaned pointclound into two planes
plane_model, inliers =downpcd_cleaned.segment_plane(distance_threshold =0.021, ransac_n = 8, num_iterations=1000)

# [a,b,c,d] = plane_model

# inliers are the "bottom" most plane
inlier_cloud = downpcd_cleaned.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0.0,0.0,1.0])

# outliers is the next highest plane, selected outliers with invert = True
# outlier is the work piece plane
outlier_cloud = downpcd_cleaned.select_by_index(inliers, invert =True)
outlier_cloud.paint_uniform_color([0.0, 0.1, 0.0])
o3d.visualization.draw_geometries([outlier_cloud, inlier_cloud],point_show_normal = False)
# Commented out lines below are writting the outlier points into a seperate point cloud
#o3d.io.write_point_cloud("work_piece.ply", outlier_cloud)

# #Reading newly saved point cloud and saving it to the variable pcd
#pcd = o3d.io.read_point_cloud("work_piece.pcd")
#o3d.visualization.draw_geometries([pcd],point_show_normal = True)

# %% Removing "noise" from work piece plane
# Finding outliers using statiscally outlier
cl, ind =outlier_cloud.remove_statistical_outlier(nb_neighbors=50, std_ratio=0.45)
filtered_outlier_cloud = display_inlier_outlier(outlier_cloud, ind)

# %%
# Cleaning up filtered outlier data to isolate workpiece
cl, ind =filtered_outlier_cloud.remove_statistical_outlier(nb_neighbors=50, std_ratio=10)
isolated_wp_cloud = display_inlier_outlier(filtered_outlier_cloud, ind)


# %%
# Rotating the filtered outlier data to be "looking down"
R = isolated_wp_cloud.get_rotation_matrix_from_xyz((0, np.pi,  0))
xyz = isolated_wp_cloud.rotate(R, center=(0,0,0))
o3d.visualization.draw_geometries([isolated_wp_cloud], point_show_normal= False)
#o3d.io.write_point_cloud("out.pcd", xyz)


# %%
# Futher isolating work piece (wp)

# Sphere center and radius
center = np.array([-0.011, -0.01, 0.000])
radius = 0.6095

points = np.asarray(isolated_wp_cloud.points)

# Calculate distances to center, set new points
distances = np.linalg.norm(points - center, axis=1)
isolated_wp_cloud.points = o3d.utility.Vector3dVector(points[distances <= radius])
o3d.visualization.draw_geometries([isolated_wp_cloud], point_show_normal= True)
