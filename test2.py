

import numpy as np

import open3d as o3d  
import matplotlib.pyplot as plt


pcd = o3d.io.read_point_cloud("copy_of_fragment.ply")
o3d.visualization.draw_geometries([pcd],point_show_normal = True)

#remove outlier

def display_inlier_outlier(cloud, ind):
    inlier_cloud =cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert =True)

    print("showing otliers(red) and inliers(Blue): ")
    outlier_cloud.paint_uniform_color([1,0,0])
    inlier_cloud.paint_uniform_color([0.0,0.0,0.1])
    #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    return inlier_cloud

print("Statistical outlier removal")
cl, ind =pcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=0.45)
filtered_pcd1 = display_inlier_outlier(pcd, ind)


cl, ind =filtered_pcd1.remove_statistical_outlier(nb_neighbors=50, std_ratio=10)
processed_pcd = display_inlier_outlier(filtered_pcd1, ind)

o3d.visualization.draw_geometries([processed_pcd])

#rotating the pointcloud

R = processed_pcd.get_rotation_matrix_from_xyz((0, np.pi,  0))
xyz = processed_pcd.rotate(R, center=(0,0,0))
o3d.visualization.draw_geometries([processed_pcd], point_show_normal= True)
o3d.io.write_point_cloud("out.pcd", xyz)

