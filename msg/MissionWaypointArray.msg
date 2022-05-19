Header header

# waypoints
MissionWaypoint[] waypoints

# action
uint8 action

# action definition
uint8 CLEAR  = 0
uint8 APPEND = 1
uint8 INSERT = 2

# buffer position (index) in case of insert
uint8 idx

# defines if the position given is global (should not effect the yaw though)
bool is_global # deprecated, will be removed soon

uint8 reference

uint8 GLOBAL    = 0 # in global frame
uint8 LOCAL     = 1 # relative to starting position and yaw
uint8 CUR_POS   = 2 # relative to current position
uint8 CUR_POSE  = 3 # relative to current position and yaw
