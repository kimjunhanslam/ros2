# test code
    
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
import open3d as o3d
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.callback,
            10  # QoS profile depth
        )
        # self.subscription  # prevent unused variable warning
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()
        
    def callback(self, msg):
    
        
        # Convert point cloud message to numpy array
        points = np.array(list(msg.data), dtype=np.float32).reshape(-1, 3)
        
        # Create Open3D point cloud & change vector to pcd
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        #down size
        pcd = pcd.voxel_down_sample(voxel_size=0.2)
        
        #outliers faile why  number of axix  = 0
        #pcd, inliers = pcd.remove_radius_outlier(nb_points=100, radius=0.5)
        
        # segment plane with RnaSAC
        plane_model, road_inliers = pcd.segment_plane(distance_threshold=0.5, ransac_n=3, num_iterations=100)
        pcd_3 = pcd.select_by_index(road_inliers, invert=True)

        
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

        
        # Update visualization
        # self.vis.clear_geometries()
        # self.vis.add_geometry(pcd)
        # self.vis.poll_events()
        # self.vis.update_renderer()
        
def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    point_cloud_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()