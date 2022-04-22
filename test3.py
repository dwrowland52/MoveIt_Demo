
import numpy as np

import open3d as o3d  
import matplotlib.pyplot as plt


pcd1 = o3d.io.read_point_cloud("out.pcd")
# o3d.visualization.draw_geometries([pcd1],point_show_normal = True)
# Sphere center and radius
center = np.array([-0.011, -0.01, 0.000])
radius = 0.6095

points = np.asarray(pcd1.points)

# Calculate distances to center, set new points
distances = np.linalg.norm(points - center, axis=1)
pcd1.points = o3d.utility.Vector3dVector(points[distances <= radius])
o3d.visualization.draw_geometries([pcd1], point_show_normal= True)

pcd_array = np.asarray(pcd1)
print(pcd_array)

pcd1.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.1, max_nn =30))
print("Print a normal of the first 10th point")
print(np.asarray(pcd1.normals)[:200, :])
# Write point cloud out
o3d.io.write_point_cloud("out2.pcd", pcd1)
