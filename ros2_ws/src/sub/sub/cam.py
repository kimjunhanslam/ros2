import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import  Image
#import sensor_msgs.point_cloud2 as pc2

# import rospy

from std_msgs.msg import String

class sub_test(Node):

    def __init__(self):
        super().__init__('cam')
        qos_profile = QoSProfile(depth = 100)
        self.subscriber_ = self.create_subscription(
            Image,
            '/front_cam',
            self.seclistener_callback,
            qos_profile
        )

    def seclistener_callback(self, msg):
        
        self.get_logger().info("cam data" + str(msg))


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