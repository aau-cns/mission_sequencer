Header header

# id is the number of the mission
uint8 id

# request
uint8 request

# request type definition
uint8 UNDEF   = 0 # Not used for request, used only for MS -> A communication without a specific request
uint8 ARM     = 1 # Arming (no takeoff)
uint8 TAKEOFF = 2 # Takeoff
uint8 HOLD    = 3 # mission hold request -- blocking, hold no until a resume request
uint8 RESUME  = 4 # resume mission request
uint8 LAND    = 5 # land request
uint8 HOVER   = 6 # hover request -- non blocking, when a new waypoint is received fly to it
uint8 ABORT   = 7 # mission aborted request - safety pilot will take manual control
uint8 DISARM  = 8 # Disarming

# deprecated values
uint8 READ    = 101 #
