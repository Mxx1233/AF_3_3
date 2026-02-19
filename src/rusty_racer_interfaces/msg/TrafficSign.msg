# Zweck: Definiert den zu erkennenden Schilder und die Distanz zwischen dem Auto und Schildern.

# Standard-Nachrichtenkopf, der einen genauen Zeitstempel und eine Koordinatensystem-ID enthalten muss.
# stamp: Zeitstempel des Eingabebildes.
# frame_id: "base_link"
std_msgs/Header header

# Es gibt insgesamt 6 Strings:
# 'end_express_way'
# 'end_zone_speed_limit'
# 'start_express_way'
# 'start_zone_speed_limit'
# 'stop_sign'
# 'yield'
string sign_id

# Die Distanz zwischen Auto und Schildern
# Einheit: Meter
float32 distance