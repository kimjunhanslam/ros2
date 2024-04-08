import open3d as o3d
import numpy as np
import time
import matplotlib.pyplot as plt
import hdbscan

import pandas as pd


class lidar_clu():

    def __init__(self):
        self.pcd = o3d.io.read_point_cloud("/home/user/ros2_ws/src/sub/sub/000000.pcd")
    
    #pcd to arr changed
    def pcdtoarr(self):
        pcd_np = np.asarray(self.pcd.points)
        print(pcd_np)
        
    #pcd to arr changed
    def arrtopcd(self):
        # numpy → point cloud
        A = np.random.random((1000, 3)) * 1000
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(A)
        
    def visual(self):
        o3d.visualization.draw_geometries([self.pcd])
        
    # pcl data dwown size
    def downsize(self):
        print(f"Points before downsampling: {len(self.pcd.points)} ")
        # Points before downsampling: 115384 
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.2)
        print(f"Points after downsampling: {len(self.pcd.points)}")
        
    # outlier removed for statical way in python
    def statical_outlier_rm(self):
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.5)
        self.pcd, inliers = self.pcd.remove_statistical_outlier(nb_points=10, radius=0.5)
        inlier_cloud = self.pcd.select_by_index(inliers)
        outlier_cloud = self.pcd.select_by_index(inliers, invert=True)
        inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        outlier_cloud.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

        self.pcd = self.pcd.select_by_index(inliers)
        
    # outlier removed for radius  way in python
    def radius_outlier_removal(self):
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.2)
        self.pcd, inliers = self.pcd.remove_radius_outlier(nb_points=50, radius=0.3)
        inlier_cloud = self.pcd.select_by_index(inliers)
        outlier_cloud = self.pcd.select_by_index(inliers, invert=True)
        inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        outlier_cloud.paint_uniform_color([1, 0, 0])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

        pcd = pcd.select_by_index(inliers)
        
    # segmentation with ransac
    def ransac_seg(self):
        
        t1 = time.time()
        plane_model, inliers = self.pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=100)
        inlier_cloud = self.pcd.select_by_index(inliers)
        outlier_cloud = self.pcd.select_by_index(inliers, invert=True)
        inlier_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        outlier_cloud.paint_uniform_color([1, 0, 0])
        t2 = time.time()
        print(f"Time to segment points using RANSAC {t2 - t1}")
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
        
    def rm_road(self):
        plane_model, road_inliers = self.pcd.segment_plane(distance_threshold=0.3, ransac_n=3, num_iterations=100)
        self.pcd = self.pcd.select_by_index(road_inliers, invert=True)
        o3d.visualization.draw_geometries([self.pcd])
    
    #clustering  dbscan 
    def dbscan(self):
        
        #black parts isn't clustered
        # CLUSTERING WITH DBSCAN
        t3 = time.time()
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(self.pcd.cluster_dbscan(eps=0.60, min_points=30, print_progress=False))

        max_label = labels.max()
        print(f'point cloud has {max_label + 1} clusters')
        colors = plt.get_cmap("tab20")(labels / max_label if max_label > 0 else 1)
        colors[labels < 0] = 0
        self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        t4 = time.time()
        print(f'Time to cluster outliers using DBSCAN {t4 - t3}')
        o3d.visualization.draw_geometries([self.pcd])
    
    #hdbscan  dbscan 
    def hdbscan(self):
        #black parts isn't clustered
        # CLUSTERING WITH HDBSCAN
        t3 = time.time()
        clusterer = hdbscan.HDBSCAN(min_cluster_size=30, gen_min_span_tree=True)
        clusterer.fit(np.array(self.pcd.points))
        labels = clusterer.labels_

        max_label = labels.max()
        print(f'point cloud has {max_label + 1} clusters')
        colors = plt.get_cmap("tab20")(labels / max_label if max_label > 0 else 1)
        colors[labels < 0] = 0
        self.pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        t4 = time.time()
        print(f'Time to cluster outliers using HDBSCAN {t4 - t3}')
        o3d.visualization.draw_geometries([self.pcd])
        
    # def bounding_box(self):
    #     bbox_objects = []
    #     indexes = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

    #     MAX_POINTS = 300
    #     MIN_POINTS = 50

    #     for i in range(0, len(indexes)):
    #         nb_points = len(pcd.select_by_index(indexes[i]).points)
    #         if (nb_points > MIN_POINTS and nb_points < MAX_POINTS):
    #             sub_cloud = pcd.select_by_index(indexes[i])
    #             bbox_object = sub_cloud.get_axis_aligned_bounding_box()
    #             bbox_object.color = (0, 0, 1)
    #             bbox_objects.append(bbox_object)

    #     print("Number of Boundinb Box : ", len(bbox_objects))

    #     list_of_visuals = []
    #     list_of_visuals.append(pcd)
    #     list_of_visuals.extend(bbox_objects)  

    #     o3d.visualization.draw_geometries(list_of_visuals)
        
    def total_try(self):
        
        pcd_path = "/home/user/ros2_ws/src/sub/sub/000000.pcd"

        pcd = o3d.io.read_point_cloud(pcd_path)

        # downsampling
        pcd_1 = pcd.voxel_down_sample(voxel_size=0.1)
        
        # remove outliers
        pcd_2, inliers = pcd_1.remove_radius_outlier(nb_points=20, radius=0.3)
        
        # segment plane with RANSAC
        plane_model, road_inliers = pcd_2.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=100)
        pcd_3 = pcd_2.select_by_index(road_inliers, invert=True)

        # CLUSTERING WITH HDBSCAN
        import matplotlib.pyplot as plt
        import hdbscan

        clusterer = hdbscan.HDBSCAN(min_cluster_size=30, gen_min_span_tree=True)
        clusterer.fit(np.array(pcd_3.points))
        labels = clusterer.labels_

        max_label = labels.max()
        print(f'point cloud has {max_label + 1} clusters')
        colors = plt.get_cmap("tab20")(labels / max_label if max_label > 0 else 1)
        colors[labels < 0] = 0
        pcd_3.colors = o3d.utility.Vector3dVector(colors[:, :3])

        # generate 3D Bounding Box
        import pandas as pd
        bbox_objects = []
        indexes = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

        MAX_POINTS = 300
        MIN_POINTS = 50

        for i in range(0, len(indexes)):
            nb_points = len(pcd_3.select_by_index(indexes[i]).points)
            if (nb_points > MIN_POINTS and nb_points < MAX_POINTS):
                sub_cloud = pcd_3.select_by_index(indexes[i])
                bbox_object = sub_cloud.get_axis_aligned_bounding_box()
                bbox_object.color = (0, 0, 1)
                bbox_objects.append(bbox_object)

        print("Number of Boundinb Box : ", len(bbox_objects))

        list_of_visuals = []
        list_of_visuals.append(pcd_3)
        list_of_visuals.extend(bbox_objects)
        # o3d.visualization.draw_geometries([pcd])
        o3d.visualization.draw_geometries(list_of_visuals)


def main(args=None):
    
    lidar = lidar_clu()
    #lidar.visual()
    #lidar.pcdtoarr()
    #lidar.downsize()
    #lidar.radius_outlier_removal()
    
    #lidar.ransac_seg()
    #lidar.dbscan()
    #lidar.hdbscan()
    lidar.total_try()
    
if __name__ == "__main__":
    main()
    
 