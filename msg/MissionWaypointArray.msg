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
bool is_global
