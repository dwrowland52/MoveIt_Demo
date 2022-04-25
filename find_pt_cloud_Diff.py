def find_Differences(file_name_PCD1, file_name_PCD2):
    #Assuming Files are in the working directory

    # Imports
    import open3d as o3d
    import numpy as np
    import matplotlib.pyplot as plt

    #Create PCD Objs
    pcd_defect = o3d.io.read_point_cloud(file_name_PCD1)   # reading point cloud file
    pcd_clean = o3d.io.read_point_cloud(file_name_PCD2)    # reading point cloud file

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

    return defect_loc