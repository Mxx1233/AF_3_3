#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import message_filters  # Wichtig für die Synchronisation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os
import datetime

# Import der eigenen Nachrichtentypen
from rusty_racer_interfaces.msg import LaneDeviation, MotorCommand

# ================= KONFIGURATION =================
# Stellen Sie sicher, dass die Topic-Namen korrekt sind
TOPIC_IMAGE_RAW    = "/camera/camera/color/image_raw"
TOPIC_IMAGE_DEBUG  = "/lane_detection/debug"    # Neu
TOPIC_IMAGE_OVERLAY= "/trajectory/overlay"      # Neu
TOPIC_DEVIATION    = "/lane_deviation"
TOPIC_COMMAND      = "/motor_command"

SAVE_DIR_ROOT = "my_dataset" # Speicherort
# =================================================

class StreamRecorder(Node):
    def __init__(self):
        super().__init__('stream_recorder')

        # 1. Erstellen des Ordners für die aktuelle Sitzung (Zeitstempel)
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.join(SAVE_DIR_ROOT, timestamp_str)
        self.img_dir = os.path.join(self.save_dir, "images")
        
        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)

        self.get_logger().info(f"== Aufnahme gestartet ==")
        self.get_logger().info(f"Daten werden gespeichert unter: {self.save_dir}")

        # 2. Initialisierung der CSV-Datei
        self.csv_file = open(os.path.join(self.save_dir, "data.csv"), 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        
        # Kopfzeile schreiben (Erweitert um neue Bilder und Latenz)
        self.writer.writerow([
            'Index',            # Laufende Nummer
            'Timestamp_Sec',    # Sekunden
            'Timestamp_Nano',   # Nanosekunden
            'Latency_ms',       # Gesamtlatenz in Millisekunden
            'Lateral_Error',    # Seitliche Abweichung
            'Heading_Error',    # Gierwinkel-Abweichung
            'Curvature',        # Krümmung
            'Steering_Angle',   # Lenkwinkel
            'Motor_Level',      # Motorleistung
            'Img_Raw',          # Dateiname Rohbild
            'Img_Debug',        # Dateiname Debugbild
            'Img_Overlay'       # Dateiname Overlaybild
        ])

        # 3. Werkzeuge initialisieren
        self.bridge = CvBridge()
        self.counter = 0

        # 4. Abonnenten einrichten (Subscriber)
        # Wir nutzen message_filters.Subscriber für die Synchronisation
        sub_raw     = message_filters.Subscriber(self, Image, TOPIC_IMAGE_RAW)
        sub_debug   = message_filters.Subscriber(self, Image, TOPIC_IMAGE_DEBUG)
        sub_overlay = message_filters.Subscriber(self, Image, TOPIC_IMAGE_OVERLAY)
        sub_dev     = message_filters.Subscriber(self, LaneDeviation, TOPIC_DEVIATION)
        sub_cmd     = message_filters.Subscriber(self, MotorCommand, TOPIC_COMMAND)

        # 5. Zeitsynchronisation (ApproximateTimeSynchronizer)
        # Wir synchronisieren nun 5 Topics gleichzeitig.
        # queue_size: Puffergröße
        # slop: Erlaubte Zeitabweichung in Sekunden (0.1s ist üblich für Header Propagation)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_raw, sub_debug, sub_overlay, sub_dev, sub_cmd], 
            queue_size=30, 
            slop=0.15 
        )
        self.ts.registerCallback(self.sync_callback)

    def sync_callback(self, msg_raw, msg_debug, msg_overlay, msg_dev, msg_cmd):
        """
        Diese Funktion wird nur aufgerufen, wenn ALLE 5 Nachrichten
        einen passenden Zeitstempel haben.
        """
        try:
            # --- 1. Latenz berechnen ---
            # Differenz zwischen "Jetzt" (Speicherzeitpunkt) und "Header" (Aufnahmezeitpunkt)
            now = self.get_clock().now()
            # Konvertierung des ROS-Time Objekts der Nachricht in Nanosekunden
            msg_time_ns = msg_raw.header.stamp.sec * 1e9 + msg_raw.header.stamp.nanosec
            current_time_ns = now.nanoseconds
            
            latency_ms = (current_time_ns - msg_time_ns) / 1e6

            # --- 2. Bilder konvertieren und speichern ---
            # Dateinamen definieren
            fname_raw     = f"raw_{self.counter:06d}.jpg"
            fname_debug   = f"debug_{self.counter:06d}.jpg"
            fname_overlay = f"overlay_{self.counter:06d}.jpg"

            # Konvertieren (ROS -> OpenCV) und Speichern
            cv_raw     = self.bridge.imgmsg_to_cv2(msg_raw, desired_encoding='bgr8')
            cv_debug   = self.bridge.imgmsg_to_cv2(msg_debug, desired_encoding='bgr8')
            cv_overlay = self.bridge.imgmsg_to_cv2(msg_overlay, desired_encoding='bgr8')

            cv2.imwrite(os.path.join(self.img_dir, fname_raw), cv_raw)
            cv2.imwrite(os.path.join(self.img_dir, fname_debug), cv_debug)
            cv2.imwrite(os.path.join(self.img_dir, fname_overlay), cv_overlay)

            # --- 3. Daten in CSV schreiben ---
            self.writer.writerow([
                self.counter,
                msg_raw.header.stamp.sec,
                msg_raw.header.stamp.nanosec,
                f"{latency_ms:.2f}",           # Latenz
                f"{msg_dev.lateral_error:.4f}",
                f"{msg_dev.heading_error:.4f}",
                f"{msg_dev.curvature:.4f}",
                f"{msg_cmd.steering_angle:.4f}",
                f"{msg_cmd.motor_level:.2f}",
                fname_raw,
                fname_debug,
                fname_overlay
            ])

            # --- 4. Konsolenausgabe (Log) ---
            # Nur jedes 10. Frame ausgeben, um die Konsole nicht zu überfluten
            if self.counter % 10 == 0:
                self.get_logger().info(
                    f"Frame [{self.counter}] gespeichert. Latenz: {latency_ms:.1f}ms"
                )

            self.counter += 1

        except Exception as e:
            self.get_logger().error(f"Fehler beim Speichern der Daten: {e}")

    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info("CSV-Datei geschlossen.")

def main(args=None):
    rclpy.init(args=args)
    node = StreamRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()