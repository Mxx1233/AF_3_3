import cv2
from ultralytics import YOLO
import time
import numpy as np

# Kein (Node) mehr, dies ist nun eine reine Logik-Klasse
class TrafficSignDetector:
    def __init__(self, model_path, conf_th=0.35, iou_th=0.45, imgsz=512, device='cpu'):
        """
        Initialisiert den Traffic Sign Detektor Operator.

        Args:
            model_path (str): Pfad zur YOLO .pt Datei.
            conf_th (float): Confidence Schwellenwert für die Erkennung.
            iou_th (float): IoU Schwellenwert für NMS.
            imgsz (int): Bildgröße für die Inferenz.
            device (str): Rechengerät ('cpu' oder '0' für GPU).
        """
        # 1. Speichern der Konfigurationsparameter
        self.conf_th = conf_th
        self.iou_th = iou_th
        self.imgsz = imgsz
        self.device = device

        # 2. Laden des Modells (Einmalig beim Initialisieren)
        # Dies ist rechenintensiv und sollte nicht im Loop gemacht werden
        print(f"Lade Modell von: {model_path}")
        self.model = YOLO(model_path)

        # Hinweis: Hier gibt es keine Publisher, Subscriber oder Timer mehr!
        # Diese Logik-Klasse "wartet" nur darauf, dass eine Funktion aufgerufen wird.

    def process(self, color_frame, depth_frame):
        """
        Führt die Erkennung und Distanzberechnung durch.

        Args:
            color_frame: RGB-Bild von der Kamera (OpenCV BGR).
            depth_frame: Ausgerichtetes Tiefenbild (16-bit uint, mm).

        Returns:
            list: Eine Liste mit Dictionaries für jedes erkannte Schild.
            numpy.ndarray: Das annotierte Debug-Bild.
        """
        # --- Zeitmessung für Inferenz-Statistiken ---
        t_start = time.time()

        # 1. YOLO Inferenz ausführen
        # Wir nutzen die Parameter, die im __init__ gespeichert wurden
        results = self.model.predict(
            source=color_frame,
            conf=self.conf_th,
            iou=self.iou_th,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False
        )[0]

        # Inferenzzeit in Millisekunden berechnen
        infer_ms = (time.time() - t_start) * 1000.0

        detections_list = []
        names = results.names

        # 2. Ergebnisse verarbeiten
        for b in results.boxes:
            # Klasse und Konfidenz
            cls_id = int(b.cls.item())
            score = float(b.conf.item())

            # --- Berechnung für Detection2D (Original-Format) ---
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            cx, cy = (x1 + x2)/2.0, (y1 + y2)/2.0
            w, h = (x2 - x1), (y2 - y1)

            # --- Robuste Distanzmessung (Median-Filter) ---
            # Wir nehmen einen 3x3 Bereich um das Zentrum, um Rauschen zu vermeiden
            ix, iy = int(cx), int(cy)
            h_img, w_img = depth_frame.shape

            # Bereich sicherstellen (ROI)
            y_min, y_max = max(0, iy-1), min(h_img, iy+2)
            x_min, x_max = max(0, ix-1), min(w_img, ix+2)

            depth_roi = depth_frame[y_min:y_max, x_min:x_max]
            # Nur Werte > 0 berücksichtigen
            valid_depths = depth_roi[depth_roi > 0]

            if len(valid_depths) > 0:
                dist_raw = np.median(valid_depths)
                distance_m = float(dist_raw) / 1000.0
            else:
                distance_m = -1.0

            # 3. Strukturierte Daten sammeln
            detections_list.append({
                'class_id': cls_id,
                'class_name': names[cls_id],
                'score': score,
                'bbox_ros': [cx, cy, w, h], # Format für Detection2D
                'bbox_xyxy': [x1, y1, x2, y2], # Für Zeichnen
                'distance': distance_m
            })

        # 4. Debug-Bild erstellen
        # Wir nutzen die Standard-Plot-Funktion von YOLO als Basis
        annotated_frame = results.plot()

        # Zusätzlich die Distanz in das Bild schreiben
        for det in detections_list:
            if det['distance'] > 0:
                txt = f"{det['distance']:.2f}m"
                # Text an der oberen Kante des Bboxes platzieren
                cv2.putText(annotated_frame, txt, (det['bbox'][0], det['bbox'][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return {
            'detections': detections_list,
            'debug_image': annotated_frame,
            'infer_ms': infer_ms
        }