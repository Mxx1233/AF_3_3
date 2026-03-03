# Zweck: Definiert den zu erkennenden Schilder und die Distanz zwischen dem Auto und Schildern.
# Standard-Nachrichtenkopf, der einen genauen Zeitstempel und eine Koordinatensystem-ID enthalten muss.
# stamp: Zeitstempel des Eingabebildes.
# frame_id: "base_link"
std_msgs/Header header

# Zu erkennende Schilder (String-Label).
# "stop_sign"               → Stop
# "start_zone_speed_limit"  → Speed30Start
# "end_zone_speed_limit"    → Speed30End
# "start_express_way"       → HighwayStart
# "end_express_way"         → HighwayEnd
# "yield"                   → YieldSlow
# andere / leer             → Unknown
string sign_id

# Die Distanz zwischen Auto und Schildern
# Einheit: Meter
float32 distance
