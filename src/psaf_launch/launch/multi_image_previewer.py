#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2

class MultiImagePreviewer(Node):
    def __init__(self):
        super().__init__('multi_image_previewer')
        self.bridge = CvBridge()
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.topics = [
            {'in': '/camera/camera/color/image_raw', 'out': '/camera/camera/color/image_raw/preview', 'count': 0},
            {'in': '/trajectory/debug_overlay',      'out': '/trajectory/debug_overlay/preview',      'count': 0},
            {'in': '/lane_detection/debug_overlay',  'out': '/lane_detection/debug_overlay/preview',  'count': 0}
        ]

        self.pubs = {}
        self.subs = []

        for item in self.topics:
            topic_in = item['in']
            topic_out = item['out']
            self.pubs[topic_in] = self.create_publisher(CompressedImage, topic_out, qos)
            sub = self.create_subscription(
                Image, topic_in, 
                lambda msg, t=topic_in: self.process_image(msg, t), 
                qos
            )
            self.subs.append(sub)

        self.get_logger().info('Multi-Preview Node: 3-in-3-out system ACTIVE')

    def process_image(self, data, topic_name):
        item = next(x for x in self.topics if x['in'] == topic_name)
        item['count'] += 1

        if item['count'] % 6 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            small_img = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)

            msg = CompressedImage()
            msg.header = data.header
            msg.format = "jpeg"
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
            _, buffer = cv2.imencode('.jpg', small_img, encode_param)
            msg.data = buffer.tobytes()
            
            self.pubs[topic_name].publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing {topic_name}: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiImagePreviewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()