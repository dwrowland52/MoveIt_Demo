# %% Imports
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


# %% Create PCD Objs
pcd_defect = o3d.io.read_point_cloud("point_test_data2_1650316030972999.pcd")   # reading point cloud file
pcd_clean = o3d.io.read_point_cloud("point_test_data2_no_defects_1650316080972000.pcd")    # reading point cloud file

pcddown_clean = pcd_clean.voxel_down_sample(voxel_size = 0.004)
pcddown_defect = pcd_defect.voxel_down_sample(voxel_size = 0.004)
#Vis the downsized pcd data
#o3d.visualization.draw_geometries([pcddown_clean], point_show_normal =False)


# %% Cleaning input data

# Gatering PCD data points as numpy arrays
clean_pts = np.asarray(pcd_clean.points)
defect_pts = np.asarray(pcd_defect.points)
#Replacing all nan values with 0 using nan_to_num()
clean_pts = np.nan_to_num(clean_pts, nan=0)
defect_pts = np.nan_to_num(defect_pts, nan=0)

#Filling the points attribute of the empty o3d obj with the cleaned data
pcd_clean.points = o3d.utility.Vector3dVector(clean_pts)
pcd_defect.points = o3d.utility.Vector3dVector(defect_pts)


# %% Compare point clouds
pcd_Compare = pcd_clean.compute_point_cloud_distance(pcd_defect)
compare_pts = np.asarray(pcd_Compare)
ind = np.where(compare_pts > 0.001)[0]
pcd_Compare = pcd_clean.select_by_index(ind)


# %% Visual compared data
o3d.visualization.draw_geometries([pcd_Compare])


# %% Removing outlide noise
R = pcd_Compare.get_rotation_matrix_from_xyz((0, np.pi,  0))
xyz = pcd_Compare.rotate(R, center=(0,0,0))
o3d.visualization.draw_geometries([pcd_Compare], point_show_normal= False)
#o3d.io.write_point_cloud("out.pcd", xyz)


# %% Segment planes
plane_model, inliers = pcd_Compare.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

inlier_cloud = pcd_Compare.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
defect_cloud = pcd_Compare.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([defect_cloud])


# %% Futher isolating work piece (wp)

# Sphere center and radius
center = np.array([-0.011, -0.01, 0.000])
radius = 0.6

points = np.asarray(defect_cloud.points)

# Calculate distances to center, set new points
distances = np.linalg.norm(points - center, axis=1)
defect_cloud.points = o3d.utility.Vector3dVector(points[distances <= radius])

wp_defects = defect_cloud.points

o3d.visualization.draw_geometries([defect_cloud], point_show_normal= False)


# %% Clustering the defects
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(defect_cloud.cluster_dbscan(eps=0.02, min_points=75, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
defect_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([defect_cloud])


# %%
