uint64 timestamp			# time since system start (microseconds)

uint8 INVALID_RESP = 0
uint8 TIMEOUT = 1

uint8 type

uint8 cmd_code
uint8 cmd_id
uint8 cmd_arg1
uint8 cmd_arg2
# TODO: maybe add cmd_data as well

uint8 resp_code
uint8 resp_id
uint8 resp_arg1
uint8 resp_arg2
uint16 resp_data
