# CIRCUIT STATUS.MSG
# uORB message definition.
# Every .msg file describes one internal PX4 topic.  From this text,
# the build system autogenerates C and C++ headers (uORB/topics/circuit_status.h)
# and metadata used by the middleware and MAVLink log streams.
#
# Each message should include a timestamp (microseconds since boot)
# so modules can track sample timing and delays.
# Other fields are your sensor-specific data that the UAVCAN bridge will fill in.

uint64 	timestamp 	# time since system start (microseconds)

uint32 	circuit_id	# unique ID for this circuit/sensor

float32 voltage		# Voltage in Volts
float32 current		# Current in Amps

uint8 	flags		# Bitmask of status flags (over/under-voltage, etc.)
