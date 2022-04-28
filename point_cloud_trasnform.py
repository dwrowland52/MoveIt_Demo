
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
import open3d as o3d  

pcd2 = o3d.io.read_point_cloud("out2.pcd")
p = np.asarray(pcd2.points)

# ee2robot = pt.transform_from_pq(
#     np.hstack((np.array([0.4, -0.3, 0.5]),
#                pr.random_quaternion(random_state))))
# cam2robot = pt.transform_from_pq(
#     np.hstack((np.array([0.0, 0.0, 0.8]), pt.q_id)))

 
#     pr.active_matrix_from_intrinsic_euler_xyz(np.array([0.0, 0.0, -0.5])),
#     np.array([0.5, 0.1, 0.1]))

df2of = pt.transform_from_pq(np.array([-0.1, 0, 0.02, 0, 0, 0,0]))
bl2df = pt.transform_from_pq(np.array([0.126918, -0.467213, 0.624013, 0.721838, -0.691879, -0.00635957, -0.0145864]))

tm = TransformManager()
tm.add_transform("depth_frame", "optical_frame", df2of)
tm.add_transform("base_link", "depth_frame", bl2df)


of2bl = tm.get_transform("optical_frame", "base_link")
Tf= np.asarray(of2bl)

point =[]
for i in range(len(p)):
    po = np.asarray(np.insert(p[i],3,0))
    prs = np.reshape(po,(4,1))
    p_tf = np.dot(Tf,prs)
    p_tfrs = np.delete(np.reshape(p_tf,(1,4)),3)
    print(p_tfrs)
    point.append(p_tfrs)

pcd3=np.asarray(point) 
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd3)
o3d.visualization.draw_geometries([pcd], point_show_normal= False)




ax = tm.plot_frames_in("base_link", s=0.1)
ax.set_xlim((-0.25, 0.75))
ax.set_ylim((-0.5, 0.5))
ax.set_zlim((0.0, 1.0))
plt.show()