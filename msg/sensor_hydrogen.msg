# Temperature sensor message

uint64 timestamp_sample         # timestamp of the sensor data (given as time since system start in microseconds)
uint64 timestamp		# timestamp when the data was published
uint32 device_id                # unique device ID for the sensor that does not change between power cycles
float32 hydrogen_percentage1     # percentage of hydrogen in the air
float32 hydrogen_percentage2      # percentage of hydrogen in the air
