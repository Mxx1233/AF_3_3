#!/usr/bin/env python3
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk
import cv2
import csv
import os


class LogViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Rusty Racer Daten-Analyse")
        self.root.geometry("1100x800")

        # Statusvariablen
        self.data_rows = []      # CSV Daten
        self.base_dir = ""       # Datenverzeichnis
        self.current_index = 0   # Aktueller Frame
        self.total_frames = 0
        self.view_mode = tk.StringVar(value="Img_Raw")  # Standardansicht

        # ================= UI LAYOUT =================

        # 1. Obere Werkzeugleiste
        top_bar = tk.Frame(root, pady=5)
        top_bar.pack(side=tk.TOP, fill=tk.X)

        tk.Button(
            top_bar,
            text="📂 Ordner öffnen",
            command=self.load_folder,
            font=(
                "Arial",
                11)).pack(
            side=tk.LEFT,
            padx=10)
        self.lbl_folder_name = tk.Label(
            top_bar, text="Kein Ordner geladen", fg="gray")
        self.lbl_folder_name.pack(side=tk.LEFT)

        # Ansichtsumschalter (Radiobuttons)
        tk.Label(
            top_bar,
            text="| Ansicht: ",
            font=(
                "Arial",
                10,
                "bold")).pack(
            side=tk.LEFT,
            padx=10)
        tk.Radiobutton(
            top_bar,
            text="Original",
            variable=self.view_mode,
            value="Img_Raw",
            command=self.refresh_image).pack(
            side=tk.LEFT)
        tk.Radiobutton(
            top_bar,
            text="Debug",
            variable=self.view_mode,
            value="Img_Debug",
            command=self.refresh_image).pack(
            side=tk.LEFT)
        tk.Radiobutton(
            top_bar,
            text="Overlay",
            variable=self.view_mode,
            value="Img_Overlay",
            command=self.refresh_image).pack(
            side=tk.LEFT)

        # 2. Hauptbereich
        main_pane = tk.PanedWindow(root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Links: Bildbereich
        self.img_frame = tk.Frame(main_pane, bg="black", width=720, height=540)
        self.img_label = tk.Label(self.img_frame, bg="black", text="Kein Bild")
        self.img_label.pack(expand=True, fill=tk.BOTH)
        main_pane.add(self.img_frame)

        # Rechts: Datenbereich
        self.data_frame = tk.Frame(main_pane, padx=20, width=300)
        main_pane.add(self.data_frame)

        # Initialisierung der Datenfelder
        self.vars = {}
        self.create_data_field("Timestamp", "Zeitstempel:")
        self.create_data_field("Latency_ms", "Latenz (ms):", color="red")

        ttk.Separator(
            self.data_frame,
            orient='horizontal').pack(
            fill='x',
            pady=10)

        self.create_data_field(
            "Lateral_Error",
            "Abweichung (m):",
            color="blue",
            font_size=14)
        self.create_data_field("Heading_Error", "Winkel (rad):")
        self.create_data_field("Curvature", "Krümmung (1/m):")

        ttk.Separator(
            self.data_frame,
            orient='horizontal').pack(
            fill='x',
            pady=10)

        self.create_data_field(
            "Steering_Angle",
            "Lenkbefehl:",
            color="green",
            font_size=14)
        self.create_data_field("Motor_Level", "Motorleistung:")

        ttk.Separator(
            self.data_frame,
            orient='horizontal').pack(
            fill='x',
            pady=10)

        self.create_data_field("Index", "Frame Nr.:")
        self.create_data_field("Filename", "Datei:")

        # 3. Untere Steuerleiste
        ctrl_bar = tk.Frame(root, pady=10, padx=10)
        ctrl_bar.pack(side=tk.BOTTOM, fill=tk.X)

        tk.Button(
            ctrl_bar,
            text="<< Zurück",
            command=self.prev_frame).pack(
            side=tk.LEFT)
        self.slider = tk.Scale(
            ctrl_bar,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            command=self.on_slider_drag,
            showvalue=0)
        self.slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        tk.Button(
            ctrl_bar,
            text="Weiter >>",
            command=self.next_frame).pack(
            side=tk.LEFT)

        # Tastatursteuerung
        root.bind('<Left>', lambda e: self.prev_frame())
        root.bind('<Right>', lambda e: self.next_frame())
        root.bind('<space>', lambda e: self.next_frame())

    def create_data_field(self, key, label_text, color="black", font_size=11):
        """Hilfsfunktion zum Erstellen von Labels"""
        frame = tk.Frame(self.data_frame)
        frame.pack(fill=tk.X, pady=2)

        tk.Label(
            frame,
            text=label_text,
            font=(
                "Arial",
                10),
            fg="gray",
            anchor="w").pack(
            fill=tk.X)
        str_var = tk.StringVar(value="--")
        self.vars[key] = str_var
        tk.Label(
            frame,
            textvariable=str_var,
            font=(
                "Consolas",
                font_size,
                "bold"),
            fg=color,
            anchor="w").pack(
            fill=tk.X)

    def load_folder(self):
        folder_path = filedialog.askdirectory(title="Datenordner auswählen")
        if not folder_path:
            return

        csv_path = os.path.join(folder_path, "data.csv")

        if not os.path.exists(csv_path):
            messagebox.showerror("Fehler", "data.csv nicht gefunden!")
            return

        self.data_rows.clear()
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.data_rows.append(row)
        except Exception as e:
            messagebox.showerror("Fehler", f"CSV Lesefehler: {e}")
            return

        if not self.data_rows:
            messagebox.showinfo("Leer", "Die CSV-Datei enthält keine Daten.")
            return

        self.base_dir = folder_path
        self.total_frames = len(self.data_rows)
        self.slider.config(to=self.total_frames - 1)
        self.lbl_folder_name.config(
            text=f"{
                os.path.basename(folder_path)} ({
                self.total_frames} Frames)")
        self.update_display(0)

    def update_display(self, index):
        if not self.data_rows:
            return
        index = max(0, min(index, self.total_frames - 1))
        self.current_index = index
        self.slider.set(index)

        row_data = self.data_rows[index]

        # 1. Daten aktualisieren
        # Mapping der CSV-Spaltennamen auf unsere internen Variablen
        mapping = {
            "Latency_ms": "Latency_ms",
            "Lateral_Error": "Lateral_Error",
            "Heading_Error": "Heading_Error",
            "Curvature": "Curvature",
            "Steering_Angle": "Steering_Angle",
            "Motor_Level": "Motor_Level",
            "Index": "Index"
        }

        for csv_key, var_key in mapping.items():
            if csv_key in row_data:
                self.vars[var_key].set(row_data[csv_key])

        # Zeitstempel formatieren
        ts = f"{
            row_data.get(
                'Timestamp_Sec',
                0)}.{
            row_data.get(
                'Timestamp_Nano',
                0)}"
        self.vars["Timestamp"].set(ts)

        # 2. Bild aktualisieren
        self.refresh_image()

    def refresh_image(self):
        """Lädt das Bild basierend auf dem aktuellen Index und dem ausgewählten Modus"""
        if not self.data_rows:
            return

        row_data = self.data_rows[self.current_index]
        mode = self.view_mode.get()  # 'Img_Raw', 'Img_Debug' oder 'Img_Overlay'

        img_name = row_data.get(mode)
        self.vars["Filename"].set(img_name)

        if img_name:
            img_path = os.path.join(self.base_dir, "images", img_name)
            if os.path.exists(img_path):
                self.show_image(img_path)
            else:
                self.img_label.config(
                    text=f"Datei fehlt: {img_name}", image="")
        else:
            self.img_label.config(text="Kein Bild für diesen Modus", image="")

    def show_image(self, path):
        cv_img = cv2.imread(path)
        if cv_img is None:
            return

        # Skalierung (Proportional)
        h, w = cv_img.shape[:2]
        target_w = 720
        ratio = target_w / float(w)
        target_h = int(h * ratio)

        cv_img = cv2.resize(cv_img, (target_w, target_h))
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        pil_img = Image.fromarray(cv_img)
        tk_img = ImageTk.PhotoImage(pil_img)

        self.img_label.config(image=tk_img)
        self.img_label.image = tk_img

    def next_frame(self):
        self.update_display(self.current_index + 1)

    def prev_frame(self):
        self.update_display(self.current_index - 1)

    def on_slider_drag(self, value):
        self.update_display(int(value))


if __name__ == "__main__":
    root = tk.Tk()
    app = LogViewer(root)
    root.mainloop()
