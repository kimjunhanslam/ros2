import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USB_Camera(Node):
    
    def __init__(self):
        
        super().__init__('usb_camera')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        # 0 cam index
        self.capture = cv2.VideoCapture(0) 
        
        # 이미지 캡쳐 및 퍼블리싱 시작
        self.timer = self.create_timer(0.1, self.capture_and_publish)

    def capture_and_publish(self):
        ret, frame = self.capture.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame)
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = USB_Camera()
    rclpy.spin(camera_node)
    camera_node.capture.release()
    camera_node.destroy_node()
    rclpy.shutdown()
    
    

if __name__ == '**main**':
    main()
    


