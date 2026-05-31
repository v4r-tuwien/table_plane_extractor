#!/usr/bin/python3

from math import atan, pi
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def extract_table_planes_from_pcd(
        pcd, 
        cluster_dbscan_eps = 0.15, 
        min_cluster_size = 200, 
        distance_threshold = 0.01, 
        max_angle_deg = 5, 
        z_min = 0.2, 
        num_iter_ransac = 1000,
        min_pre_cluster_size = 150):
    '''
    Extract possible horizontal planes from the point cloud.
    
    Parameters:
        pcd (open3d.geometry.PointCloud): Input point cloud without NANs and with Z pointing up.
        cluster_dbscan_eps (float): DBSCAN clustering epsilon value.
        min_cluster_size (int): Minimum number of points required to form a cluster.
        distance_threshold (float): RANSAC distance threshold for plane segmentation.
        max_angle_deg (float): Maximum angle (in degrees) allowed for a plane to be considered horizontal.
        z_min (float): Maximum Z coordinate value for a point to be considered as part of the floor.
        
    Returns:
        list: List of tuples (a, b, c, d) representing plane equations (a * x + b * y + c * z + d = 0).
        list: List of oriented bounding boxes (open3d.geometry.OrientedBoundingBox) around the detected planes.
    '''

    planes = []
    bboxes = []

    max_angle_rad = max_angle_deg * pi/180

    points = np.array(pcd.points)
    floor = points[:, 2] < z_min
    pcd.points = o3d.utility.Vector3dVector(points[floor == 0])


    # Cluster scene after floor removal to get rough semantic seperation
    # for more precise plane segmentation (otherwise points of similar 
    # hight scatterewd over the whole scene influence the plane segmentation)
    label_vec = np.array(pcd.cluster_dbscan(
        eps=cluster_dbscan_eps, 
        min_points=min_pre_cluster_size))

    # #Visualize the clustered point cloud in Open3D
    # max_label = label_vec.max()
    # colors = plt.cm.get_cmap("tab20")(label_vec / (max_label if max_label > 0 else 1))
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([pcd])

    labels = np.unique(label_vec)

    # detect planes for every cluster
    for label in labels:
        if label == -1:
            continue

        cluster_idx = np.where(np.asarray(label_vec) == label)[0]
        cluster = pcd.select_by_index(cluster_idx)

        cluster_pcd = cluster
        while len(cluster_pcd.points) > min_cluster_size:
            plane_model, inliers = cluster_pcd.segment_plane(
                distance_threshold=distance_threshold,
                ransac_n=3,
                num_iterations=num_iter_ransac)
            
            # ax + by + cz + d = 0
            [a, b, c, d] = plane_model

            # remove non-horizontal-planes
            # z = (-ax - by - d)/c -> gradient = (-a/c, -b/c)
            if (abs(atan(-a/c)) > max_angle_rad or abs(atan(-b/c)) > max_angle_rad):
                cluster_pcd = cluster_pcd.select_by_index(inliers, invert=True)
                continue

            inlier_cloud = cluster_pcd.select_by_index(inliers)
            cluster_pcd = cluster_pcd.select_by_index(inliers, invert=True)
            # segment plane-pointcloud because multiple objects could potentially align with the same plane
            idx = inlier_cloud.cluster_dbscan(cluster_dbscan_eps, min_cluster_size)
            vals = np.unique(idx)
            for val in vals:
                if val == -1:
                    continue
                # select clustered plane cloud
                cluster_idx = np.where(np.asarray(idx) == val)[0]
                plane_pc = inlier_cloud.select_by_index(cluster_idx)
                #o3d.visualization.draw_geometries([plane_pc])
                bb_plane = plane_pc.get_oriented_bounding_box()
                planes.append((a,b,c,d))

                print("Plane equation: {}x + {}y + {}z + {} = 0".format(a, b, c, d))
                bb_plane = plane_pc.get_minimal_oriented_bounding_box(robust=True)
                bboxes.append(bb_plane)
    if len(planes) == 0:
        print("No planes found")
        return None, None
    
    return planes, bboxes


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("/root/HSR/catkin_ws/test_case.pcd")
    print("Loaded point cloud with {} points".format(len(pcd.points)))

    cluster_dbscan_eps = 0.15
    min_cluster_size = 200
    distance_threshold = 0.01
    max_angle_deg = 5
    z_min = 0.2
    num_iter_ransac = 1000

    plane_count = []
    for i in range(100):
        print(i)
        planes, bboxes = extract_table_planes_from_pcd(pcd, cluster_dbscan_eps, min_cluster_size, distance_threshold, max_angle_deg, z_min, num_iter_ransac)
        plane_count.append(len(planes))
    print("Average number of planes: {}".format(np.mean(plane_count)))
    print("Standard deviation: {}".format(np.std(plane_count)))
    sd = np.std(plane_count)
    mean_plane_count = np.mean(plane_count)
    unique_values, counts = np.unique(plane_count, return_counts=True)

    plt.bar(unique_values, counts)
    plt.axvline(mean_plane_count, color='r', linestyle='dashed', linewidth=2, label='Mean')
    plt.xlabel('Unique Values')
    plt.ylabel('Counts')
    plt.title('Count of Unique Values in plane_count')
    plt.legend()
    plt.savefig(f'/root/HSR/catkin_ws/planes_{cluster_dbscan_eps}_{min_cluster_size}_{distance_threshold}_{max_angle_deg}_{z_min}_{num_iter_ransac}.png')