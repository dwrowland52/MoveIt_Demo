
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
import open3d as o3d  

# opening the point cloud data
pcd2 = o3d.io.read_point_cloud("copy_of_fragment.pcd")
p = np.asarray(pcd2.points)

# Determining the known transformation matrix
of2df = pt.transform_from_pq(np.array([-0.1, 0, 0.02, 0, 0, 0,0]))
bl2df = pt.transform_from_pq(np.array([0.126918, -0.467213, 0.624013, -0.0145864, 0.721838,-0.691879, -0.00635957]))

tm = TransformManager()
tm.add_transform("optical_frame", "depth_frame", of2df)
tm.add_transform("base_link", "depth_frame", bl2df)

# Obtaining the required transformation matrix
of2bl = tm.get_transform("optical_frame", "base_link")
Tf= np.asarray(of2bl)
print(Tf)

#multiplying point cloud data with transformation matrix
point =[]
for i in range(len(p)):
    po = np.asarray(np.insert(p[i],3,1))
    
    prs = np.reshape(po,(4,1))
    
    p_tf = np.matmul(Tf,prs)
  
   
    p_tfrs = np.delete(p_tf,3)

    point.append(p_tfrs)

# output
pcd3=np.asarray(point) 



# visualization below this line
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd3)
o3d.visualization.draw_geometries([pcd], point_show_normal= False)

pq= pcd3


x=[]
y=[]
z=[]
for i in range(len(pq)):
    x.append(pq[i][0])
    y.append(pq[i][1])
    z.append(pq[i][2])


ax = tm.plot_frames_in("base_link", s=0.1)

ax.scatter(x,y,z)
plt.show()