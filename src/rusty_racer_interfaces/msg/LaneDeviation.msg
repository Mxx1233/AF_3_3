# Zweck: Beschreibt die Abweichung des Fahrzeugs von der Mitte der erkannten Fahrspur.
#        Wird vom Wahrnehmungsknoten veröffentlicht.

# Standard-Nachrichtenkopf, der einen genauen Zeitstempel und eine Koordinatensystem-ID enthalten muss.
# stamp: Zeitstempel des Eingabebildes.
# frame_id: "base_link"
std_msgs/Header header

# Laterale Abweichung in Metern (meters).
# Beschreibt die seitliche Verschiebung des Fahrzeug-Referenzpunktes (Ursprung von base_link)
# von der Mitte der Fahrspur.
# Konvention: Ein positiver Wert bedeutet, dass sich das Fahrzeug rechts von der Mitte befindet.
float32 lateral_error

# Gierwinkel-Abweichung in Radiant (radians).
# Beschreibt den Winkel zwischen der Fahrzeugausrichtung (X-Achse von base_link) und der
# Tangentenrichtung der Fahrspur.
# Konvention: Ein positiver Wert bedeutet, die Fahrzeugfront zeigt nach rechts von der Fahrspurrichtung.
float32 heading_error

# (Optionale Implementierung) Geschätzte Krümmung der Fahrspur in 1/Meter.
# Eine Gerade hat die Krümmung 0.
float32 curvature
