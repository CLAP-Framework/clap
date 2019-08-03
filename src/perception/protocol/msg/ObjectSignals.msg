# This message is used to represent detected vehicle light signals or human hand signals

# Signal flags. Multiple signal emission can exists in the same time.
uint16 flags

uint16 VALID = 1 # indicates whether message is valid
uint16 LEFT_TURN_SIGNAL = 2
uint16 RIGHT_TURN_SIGNAL = 4
uint16 BRAKE_SIGNAL = 8

# Confidence of the signal detection
float32 score
