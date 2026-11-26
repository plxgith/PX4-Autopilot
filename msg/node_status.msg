uint64 	timestamp 	# Time since system start (microseconds)
uint32 	uptime_sec 	# Time since start-up of the node (seconds)

uint8 HEALTH_OK         = 0     # The node is functioning properly.
uint8 HEALTH_WARNING    = 1     # A critical parameter went out of range or the node encountered a minor failure.
uint8 HEALTH_ERROR      = 2     # The node encountered a major failure.
uint8 HEALTH_CRITICAL   = 3     # The node suffered a fatal malfunction.
uint8 health

uint8 MODE_OPERATIONAL      = 0         # Normal operating mode.
uint8 MODE_INITIALIZATION   = 1         # Initialization is in progress; this mode is entered immediately after startup.
uint8 MODE_MAINTENANCE      = 2         # E.g. calibration, the bootloader is running, etc.
uint8 MODE_SOFTWARE_UPDATE  = 3         # New software/firmware is being loaded.
uint8 MODE_OFFLINE          = 7         # The node is no longer available.
uint8 mode

#
# Not used currently, keep zero when publishing, ignore when receiving.
#
# uint3 sub_mode


uint16 vendor_specific_status_code
