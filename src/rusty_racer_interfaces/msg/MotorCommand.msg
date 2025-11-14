# Zweck: Definiert den übergeordneten Steuerbefehl, der vom Kontrollknoten an den
#        Adapterknoten der unteren Ebene gesendet wird.
# Die Einheiten sind physikalische Einheiten, entkoppelt von der Hardware.

# Gewünschter Lenkwinkel des Servos in Radiant (radians).
# Konvention: Positive Werte bedeuten eine Rechtskurve, negative eine Linkskurve.
float32 steering_angle

# Gewünschtes Antriebsniveau des Motors als normalisierter Wert im Bereich [-1.0, 1.0].
# Konvention:
#   (0.0, 1.0] -> Vorwärtsbewegung
#   0.0        -> Null Leistung / Rollen
#   [-1.0, 0.0) -> Aktives Bremsen oder Rückwärtsfahrt.
# Für die erste Phase verwenden wir nur den Bereich [0.0, 1.0].
float32 motor_level
