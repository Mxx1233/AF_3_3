import time

from cv_bridge import CvBridge
import message_filters
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rusty_racer_interfaces.msg import TrafficSign
from sensor_msgs.msg import Image

from .traffic_sign_detector import TrafficSignDetector


class DetectTrafficSignNode(Node):
    def __init__(self):
        super().__init__('detect_traffic_sign')

        # ---- Parameter (damit du nichts hardcoden musst) ----
        self.declare_parameter('topics.image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'topics.depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter('topics.detections_topic', '/traffic_sign/detections')
        self.declare_parameter('topics.debug_image_topic', '/traffic_sign/debug_image')

        self.declare_parameter(
            'model_path', 'src/traffic_sign_yolo/model/best_run_1_long_30_01_2026.pt'
        )  # Pfad zu deinem .pt
        self.declare_parameter('conf_th', 0.35)  # Confidence threshold
        self.declare_parameter('iou_th', 0.45)  # NMS IoU
        self.declare_parameter('imgsz', 512)  # Inference size
        self.declare_parameter('device', 'cpu')  # 'cpu' oder z.B. '0' für GPU später
        self.declare_parameter('publish_debug_image', True)

        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        depth_topic = (
            self.get_parameter('depth_topic').get_parameter_value().string_value
        )
        det_topic = (
            self.get_parameter('detections_topic').get_parameter_value().string_value
        )
        dbg_topic = (
            self.get_parameter('debug_image_topic').get_parameter_value().string_value
        )
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.get_logger().info(f'Lade YOLO Modell: {model_path}')
        self.detector = TrafficSignDetector(
            model_path=self.get_parameter('model_path').value,
            conf_th=self.get_parameter('conf_th').value,
            iou_th=self.get_parameter('iou_th').value,
            imgsz=self.get_parameter('imgsz').value,
            device=self.get_parameter('device').value,
        )

        # 3. Statistik-Variablen (Der Filter aus dem Original)
        self.avg_infer_ms = 0.0
        self.avg_total_ms = 0.0
        self.t_last = time.time()
        self.n_frames = 0
        self.alpha = 0.1  # Der Filter-Koeffizient

        # 4. Synchronisierte Subscriber (RGB + Depth)
        self.sub_rgb = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data
        )
        self.sub_depth = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )

        # Synchronisator: Sucht Bilder mit ähnlichen Zeitstempeln
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth], 10, 0.1
        )
        self.ts.registerCallback(self.on_frames)

        self.pub_det = self.create_publisher(TrafficSign, det_topic, 10)
        self.pub_dbg = self.create_publisher(Image, dbg_topic, 10)

    def on_frames(self, msg_rgb, msg_depth):

        t0 = time.time()

        # 1. ROS -> OpenCV
        cv_rgb = self.bridge.imgmsg_to_cv2(msg_rgb, 'bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(msg_depth, '16UC1')

        # 2. Logik-Calculator aufrufen
        result = self.detector.process(cv_rgb, cv_depth)

        detections = result['detections']
        debug_img = result['debug_image']
        infer_ms = result['infer_ms']

        total_ms = (time.time() - t0) * 1000.0

        # 3. Statistik-Berechnung (Zur vollständigen Ausrichtung am Original)
        # Gleitender Mittelwert Filter (Alpha-Filter)
        self.avg_infer_ms = (
            1.0 - self.alpha
        ) * self.avg_infer_ms + self.alpha * infer_ms
        self.avg_total_ms = (
            1.0 - self.alpha
        ) * self.avg_total_ms + self.alpha * total_ms

        # Kamera-Latenz berechnen (Zeitpunkt der Aufnahme bis jetzt)
        stamp = (msg_rgb.header.stamp.sec
                 + msg_rgb.header.stamp.nanosec * 1e-9)
        now = time.time()
        lat_ms = (now - stamp) * 1000.0

        # FPS-Logging (Alle 1 Sekunde, wie im Original)
        self.n_frames += 1
        if now - self.t_last > 1.0:
            fps = self.n_frames / (now - self.t_last)
            self.get_logger().info(
                f'FPS={fps:.1f} '
                f'infer={self.avg_infer_ms:.1f}ms '
                f'total={self.avg_total_ms:.1f}ms '
                f'cam_latency={lat_ms:.1f}ms'
            )
            self.t_last = now
            self.n_frames = 0

        # 4. Ergebnisse publizieren
        if detections:
            closest = min(detections, key=lambda d: d['distance'])

            msg = TrafficSign()
            msg.sign_id = str(closest['class_name'])
            msg.distance = float(closest['distance'])

            self.pub_det.publish(msg)

        # 5. Debug-Bild publizieren
        if bool(self.get_parameter('publish_debug_image').value):
            dbg_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
            dbg_msg.header = msg_rgb.header
            self.pub_dbg.publish(dbg_msg)


def main():
    rclpy.init()
    node = DetectTrafficSignNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
