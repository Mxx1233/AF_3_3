---

# Team-Guide: Zentrale ROS 2 Parameter-Konfiguration (YAML)

Hallo zusammen,
unser Projekt nutzt ab sofort eine **zentrale Konfiguration** über das `psaf_launch`-Paket. 
**Der große Vorteil:** Änderungen an Topics, PIDs oder Schwellenwerten erfordern **kein neues `colcon build` mehr**! Einfach die YAML ändern, Launch-File neu starten, fertig.

*(Besonderer Hinweis an das **Control-Team**: Eure Parameter stehen bereits in der YAML, werden aber noch ignoriert, da sie im Code noch nicht deklariert wurden! Bitte lest Punkt 4 aufmerksam.)*

---

## 1. Struktur & Regeln der YAML-Datei
 **Pfad:** `src/psaf_launch/config/psaf_launch.yaml`

```yaml
/**: # Globale Konfiguration (für alle Nodes)
  ros__parameters:
    topics: # ALLE Topic-Namen kommen hier rein!
      motor_command: "/motor_command"
      image_topic: "/camera/camera/color/image_raw"

/control_node: # Node-spezifische Konfiguration
  ros__parameters:
    lat_kp: 0.75
    cruise_mode: True
```

**Was darf in die YAML?**
1. **Topics:** Immer global unter `/** -> ros__parameters -> topics`.
2. **Fine-Tuning:** Parameter, die oft angepasst werden (PID, Thresholds, Frequenzen), kommen in die **node-spezifische** Sektion.
3. **Keep it simple:** Alles andere (feststehende Konstanten) bleibt hardcodiert im Code. Die YAML soll übersichtlich bleiben.
4. **Pflicht:** Parameter müssen immer unter dem Keyword `ros__parameters:` stehen.

---

## 2. Woher kommt der Node-Name in der YAML?
Namen wie `/control_node` oder `/lane_detection_outside` beziehen sich **nicht** auf den Paketnamen oder die Executable! 
Sie entsprechen exakt dem **`name`**-Attribut, das im zentralen Launch-File (`main_psaf1.launch.py`) definiert wurde:
```python
Node(package='rusty_racer_control', executable='...', name='control_node', ...)
```

---

## 3. Die goldene Regel: Deklarieren ist Pflicht!
In ROS 2 werden Parameter aus der YAML **nicht automatisch** in das Programm geladen. 
**Jeder Parameter muss im Code aktiv deklariert werden**, sonst wird der Wert in der YAML ignoriert oder das Programm stürzt ab.

---

## 4. Implementierung im Code

### Python (z.B. Perception / Planning)
Im `__init__` eurer Node:
```python
# 1. Deklarieren & Default-Wert setzen (Vorsicht bei Floats: 30.0 statt 30)
self.declare_parameter('topics.image_topic', '/camera/image_raw') # Globales Topic
self.declare_parameter('update_frequency', 30.0)                  # Eigener Parameter

# 2. Werte auslesen (YAML überschreibt jetzt die Defaults)
img_topic = self.get_parameter('topics.image_topic').value
freq = self.get_parameter('update_frequency').value

self.create_subscription(Image, img_topic, self.callback, 10)
```

### ⚙️ C++ (z.B. Control / Low Level)
Im Konstruktor eurer Node:
```cpp
// 1. Deklarieren & Default-Wert setzen
// WICHTIG: Floats IMMER mit Nachkommastelle (1.0), sonst droht eine ParameterTypeException!
this->declare_parameter("topics.motor_command", "/motor_command");
this->declare_parameter("lat_kp", 0.5);
this->declare_parameter("cruise_mode", false);

// 2. Werte auslesen und umwandeln
std::string motor_topic = this->get_parameter("topics.motor_command").as_string();
double kp = this->get_parameter("lat_kp").as_double();
bool cruise = this->get_parameter("cruise_mode").as_bool();
```

---

## 5. Wichtig für den Workflow
1. **Bauen:** Ab sofort **immer** mit Symlinks kompilieren!
   ```bash
   colcon build --symlink-install
   ```
2. **Testen:** Checken, ob euer Parameter erkannt wurde:
   ```bash
   ros2 param get /control_node lat_kp
   ```
