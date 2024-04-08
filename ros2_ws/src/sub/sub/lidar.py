import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import  Image


from geometry_msgs.msg import PointStamped

from velodyne_msgs.msg import VelodyneScan

import pcl
#from pclpy import pcl
import numpy as np
import pcl

#import sensor_msgs.point_cloud2 as pc2
# import rospy
#from std_msgs.msg import String

class sub_test(Node):

    def __init__(self):
        super().__init__('lidar')
        qos_profile = QoSProfile(depth = 100)
        self.subscriber_ = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.seclistener_callback,
            qos_profile
        )

    def seclistener_callback(self, msg):
        
        #self.get_logger().info("velodyne_points" + str(msg.fields[0].name))
        #self.get_logger().info("velodyne_points" + str(msg.data[0]))
        
        cloud = pcl.PointCloud.PointXYZ()
        points = np.array(list(msg.data), dtype=np.float32)
        cloud.from_array(points)

        # Create a KD-Tree
        tree = cloud.make_kdtree()

        # Euclidean Clustering
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)  # Set the spatial distance threshold for clustering
        ec.set_MinClusterSize(100)      # Set the minimum number of points that a cluster should have
        ec.set_MaxClusterSize(25000)    # Set the maximum number of points that a cluster should have
        ec.set_SearchMethod(tree)
        cluster_indices = pcl.vectors.IntVector()
        ec.extract(cluster_indices)

        # Process the clusters
        for j, indices in enumerate(cluster_indices):
            cluster = pcl.PointCloud.PointXYZ()
            cluster.from_list(cloud, indices)
            # Process each cluster, e.g., publish, visualize, etc.



def main(args=None):
    rclpy.init(args=args)
    node = sub_test()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyborad Interrup")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    
    main()
    
    
#!/usr/bin/env python


# def callback(data):
#     # Parse point cloud data
#     for point in pc2.read_points(data, skip_nans=True):
#         x, y, z = point[:3]  # Extract x, y, z coordinates
#         # Do something with x, y, z...

# def listener():
#     rospy.init_node('pointcloud_listener', anonymous=True)
#     rospy.Subscriber("/cloud_point", PointCloud2, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# import pclpy
# from pclpy import pcl
# import numpy as np

# class PointCloudCluster(Node):
#     def __init__(self):
#         super().__init__('point_cloud_cluster')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/velodyne_points',
#             self.point_cloud_callback,
#             10
#         )

#     def point_cloud_callback(self, msg):
        # # Convert ROS PointCloud2 message to PCL PointCloud
        # cloud = pcl.PointCloud.PointXYZ()
        # points = np.array(list(msg.data), dtype=np.float32)
        # cloud.from_array(points)

        # # Create a KD-Tree
        # tree = cloud.make_kdtree()

        # # Euclidean Clustering
        # ec = cloud.make_EuclideanClusterExtraction()
        # ec.set_ClusterTolerance(0.02)  # Set the spatial distance threshold for clustering
        # ec.set_MinClusterSize(100)      # Set the minimum number of points that a cluster should have
        # ec.set_MaxClusterSize(25000)    # Set the maximum number of points that a cluster should have
        # ec.set_SearchMethod(tree)
        # cluster_indices = pcl.vectors.IntVector()
        # ec.extract(cluster_indices)

        # # Process the clusters
        # for j, indices in enumerate(cluster_indices):
        #     cluster = pcl.PointCloud.PointXYZ()
        #     cluster.from_list(cloud, indices)
        #     # Process each cluster, e.g., publish, visualize, etc.

# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudCluster()
#     rclpy.spin(node)
#     rclpy.shutdown()

# class PointCloudSubscriber(Node):
#     def __init__(self):
#         super().__init__('point_cloud_subscriber')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             'point_cloud_topic',
#             self.callback,
#             10  # QoS profile depth
#         )
#         self.subscription  # prevent unused variable warning
#         self.vis = o3d.visualization.Visualizer()
#         self.vis.create_window()
        
#     def callback(self, msg):
#         # Convert point cloud message to numpy array
#         points = np.array(list(msg.data), dtype=np.float32).reshape(-1, 3)
#         # Create Open3D point cloud
#         pcd = o3d.geometry.PointCloud()
#         pcd.points = o3d.utility.Vector3dVector(points)
#         # Update visualization
#         self.vis.clear_geometries()
#         self.vis.add_geometry(pcd)
#         self.vis.poll_events()
#         self.vis.update_renderer()
        
# def main(args=None):
#     rclpy.init(args=args)
#     point_cloud_subscriber = PointCloudSubscriber()
#     rclpy.spin(point_cloud_subscriber)
#     point_cloud_subscriber.destroy_node()
#     rclpy.shutdown()
    
# if __name__ == '__main__':
#     main()