uint64 timestamp
bool   valid

uint8 frame # 0 = GLOBAL (WGS84)

float64 lat
float64 lon
float32 alt_amsl

float32 approach_course # rad, 0 = North, +CW
float32 standoff_distance
float32 run_length

float32 cruise_speed
float32 acceptance_radius

uint8 flags # bit0: rtl_on_complete
