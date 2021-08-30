Header header

# id is the number of the mission
uint8 id

# request
uint8 request

# request type definition
uint8 UNDEF  = 0
uint8 READ   = 1 # mission read request
uint8 ARM    = 2 # Arming
uint8 HOLD   = 3 # mission hold request
uint8 RESUME = 4 # resume mission request
uint8 ABORT  = 5 # mission aborted request - safety pilot will take manual control
uint8 DISARM = 6 # Disarming
uint8 LAND   = 7 # Landing
