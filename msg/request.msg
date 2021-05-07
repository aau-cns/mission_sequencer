Header header

# id is the number of the mission
uint8 id

# request
uint8 request

# request type definition
uint8 UNDEF  = 0
uint8 START  = 1 # mission start request
uint8 HOLD   = 2 # mission hold request
uint8 RESUME = 3 # resume mission request
uint8 ABORT  = 4 # mission aborted request - safety pilot will take manual control
