# Zweck: Definiert den zu erkennenden Schilder und die Distanz zwischen dem Auto und Schildern.

# Standard-Nachrichtenkopf, der einen genauen Zeitstempel und eine Koordinatensystem-ID enthalten muss.
# stamp: Zeitstempel des Eingabebildes.
# frame_id: "base_link"
std_msgs/Header header

# z.B. "Stop", "Speed-limit 30"
string sign_id

# Die Distanz zwischen Auto und Schildern
# Einheit: Meter
float32 distance