bool connected
bool armed
bool estop
bool comm_timeout
bool sysid
bool uart_error
bool invalid_output
bool tx_queue_drop
bool invalid_command

uint32 status_flags
uint32 control_tick
uint16 last_setpoint_seq
uint16 feedback_sequence

uint8 firmware_major
uint8 firmware_minor
uint8 firmware_patch
uint8 protocol_version

float64 left_position_rad
float64 right_position_rad
float64 left_velocity_rad_s
float64 right_velocity_rad_s

float64 brush_right_rpm
float64 brush_left_rpm
float64 conveyor_rpm

bool collector_command_fresh
