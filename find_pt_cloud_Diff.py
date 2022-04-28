def find_Differences(file_name_Clean_PCD, file_name_Defect_PCD):
    #Assuming Files are in the working directory

    # Imports
    import open3d as o3d
    import numpy as np
    from pytransform3d import rotations as pr
    from pytransform3d import transformations as pt
    from pytransform3d.transform_manager import TransformManager

    # Transformations from: 
    df2of = pt.transform_from_pq(np.array([-0.1, 0, 0.02, 0, 0, 0,0])) #Depth Frame to Optical Frame
    bl2df = pt.transform_from_pq(np.array([0.126918, -0.467213, 0.624013, 0.721838, -0.691879, -0.00635957, -0.0145864])) #Base Link to Depth Frame

    # USing TF Manager
    tm = TransformManager()
    tm.add_transform("depth_frame", "optical_frame", df2of)
    tm.add_transform("base_link", "depth_frame", bl2df)

    # Getting total TF
    of2bl = tm.get_transform("optical_frame", "base_link")
     # Storing as np matrix
    Tf= np.asarray(of2bl)

    #Create PCD Objs
    pcd_defect = o3d.io.read_point_cloud(file_name_Defect_PCD)   # reading point cloud file
    pcd_clean = o3d.io.read_point_cloud(file_name_Clean_PCD)    # reading point cloud file

    pcddown_clean = pcd_clean.voxel_down_sample(voxel_size = 0.004)
    pcddown_clean.paint_uniform_color([1.0, 0.0, 0.0])
    pcddown_defect = pcd_defect.voxel_down_sample(voxel_size = 0.004)
    pcddown_defect.paint_uniform_color([0.0, 0.0, 1.0])
    #Vis the downsized pcd data
    o3d.visualization.draw_geometries([pcddown_clean, pcddown_defect], point_show_normal =False)

    #Cleaning input data
    # Gatering PCD data points as numpy arrays
    clean_pts = np.asarray(pcd_clean.points)
    defect_pts = np.asarray(pcd_defect.points)
    #Replacing all nan values with 0 using nan_to_num()
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
    o3d.visualization.draw_geometries([pcd_Compare])

    #Segment planes
    plane_model, inliers = pcd_Compare.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    inlier_cloud = pcd_Compare.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    defect_cloud = pcd_Compare.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([defect_cloud])

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

    o3d.visualization.draw_geometries([defect_cloud], point_show_normal= False)

    point =[]
    for i in range(len(defect_loc)):
        po = np.asarray(np.insert(defect_loc[i],3,0))
        prs = np.reshape(po,(4,1))
        p_tf = np.dot(Tf,prs)
        p_tfrs = np.delete(np.reshape(p_tf,(1,4)),3)
        print(p_tfrs)
        point.append(p_tfrs)

    defect_loc_BaseLink_frame=np.asarray(point) 

    return defect_loc_BaseLink_frame