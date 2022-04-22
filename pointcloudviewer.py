# examples/Python/Basic/pointcloud.py

from cv2 import DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING
import numpy as np
import sys
import open3d as o3d  
import matplotlib.pyplot as plt
from sympy import false




pcd = o3d.io.read_point_cloud("defects.pcd")   # reading point cloud file
print(pcd) #printing the dataset 
pcd_array = np.asarray(pcd)
print(pcd_array)
#o3d.visualization.draw_geometries([pcd])

                                               

#voxel downsampling (In 3D computer graphics, a voxel represents a value on a regular grid in three-dimensional space)
print("Downsample the point cloud with a voxel of 0.005")
downpcd = pcd.voxel_down_sample(voxel_size = 0.007)
downpcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn =30))  #Voxel Normal Estimation : Recompute the normal of the point cloud
#o3d.visualization.draw_geometries([downpcd], point_show_normal =False)

# access estimated vertex normal 
print("Print a normal of the first 10th point")
print(np.asarray(downpcd.normals)[:100, :])

#painting
print('Painting')
downpcd.paint_uniform_color([0, 0.706, 1])
# o3d.visualization.draw_geometries([downpcd], point_show_normal =False)

# 3dbounding box
aabb = downpcd.get_axis_aligned_bounding_box()
aabb.color = (1, 0, 0) #red
obb = downpcd.get_oriented_bounding_box()
obb.color = (0, 0, 1) #blue
o3d.visualization.draw_geometries([downpcd,aabb,obb], point_show_normal =False)

# dbscan clustering 
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     label =downpcd.cluster_dbscan(eps = 20, min_points= 1000, print_progress=True)
#     labels = np.array(label)

# max_label =labels.max()
# print(f"point cloud has {max_label +1} clusters")
# colors = plt.get_cmap("tab20")(labels /(max_label if max_label > 0 else 1))
# colors[labels < 0] = 0
# downpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
# o3d.visualization.draw_geometries([downpcd])

#plane segmentation
plane_model, inliers =downpcd.segment_plane(distance_threshold =0.021, ransac_n = 8, num_iterations=1000)

[a,b,c,d] =plane_model

inlier_cloud = downpcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0.0,0,1.0])
outlier_cloud = downpcd.select_by_index(inliers, invert =True)
#o3d.visualization.draw_geometries([outlier_cloud],point_show_normal = false)
o3d.io.write_point_cloud("copy_of_fragment.ply", outlier_cloud)

pcd = o3d.io.read_point_cloud("copy_of_fragment.pcd")
o3d.visualization.draw_geometries([pcd],point_show_normal = True)

#remove outlier

def display_inlier_outlier(cloud, ind):
    inlier_cloud =cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert =True)

    print("showing otliers(red) and inliers(gray): ")
    outlier_cloud.paint_uniform_color([1,0,0])
    inlier_cloud.paint_uniform_color([0.8,0.8,0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


print("Statistical outlier removal")
cl, ind =downpcd.remove_statistical_outlier(nb_neighbors=2, std_ratio=1)
display_inlier_outlier(pcd, ind)


# #radius outlier removal 
# print("radius outlier remove")
# cl, ind = downpcd.remove_radius_outlier(nb_points=16, radius=0.01)
# display_inlier_outlier(downpcd, ind)








