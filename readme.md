# AMAZE Mission sequencer


The mission sequencer has no external dependencies and listens to `/autonomy/requests` and can be lauchned via
```cmd
roslaunch amaze_mission_sequencer amaze_mission_sequencer.launch 

```


Command to set filepath to test_trajectory.csv:
```cmd
rosparam set /autonomy/missions/mission_0/filepaths ["/home/chriboehm/workspaces/mission_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv"]
```

Pattern for command for requests:
```cmd
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: #}'
```


To execute a mission, first set the file paths of mission XY and the issue the `ARM` request and finally a `LAND` or `ABORT` request.
```cmd
# read mission files specified at parameter server... 
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 1}'
# arming will execute the mission autonomously
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 2}'
# the agent won't land autonomously, one has to issue the land command!
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 7}'

``` 

The request IDs are specified in `msg/request.msg`.
```yaml
# request type definition
uint8 UNDEF  = 0
uint8 READ   = 1 # mission read request
uint8 ARM    = 2 # Arming
uint8 HOLD   = 3 # mission hold request
uint8 RESUME = 4 # resume mission request
uint8 ABORT  = 5 # mission aborted request - safety pilot will take manual control
uint8 DISARM = 6 # Disarming
uint8 LAND   = 7 # Landing
```

## Launching with one Mission file

A single mission file can be passed at startup and sets the sequencer in state `PREARM`.
```cmd
roslaunch amaze_mission_sequencer amaze_mission_sequencer.launch waypoint_fn:=/home/jungr/workspace/NAV/development/catkin_workspaces/ansible_workspace/uwb_datarecording_aws/host_basestation/ros_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv
```

To execute a mission, just issue the `ARM` request. 
If the ros param `/amaze_mission_sequencer/automatic_landing` is true, the landing command will be issued after the last waypoint was reached. After landing was detected, the sequencer will be `IDLE` again and ready to read from the mission file(s). 
```cmd
# arming will execute the mission autonomously
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 2}'
# the agent won't land autonomously, if param automatic_land = true
# otherise issue:
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 7}'


# once the agent is landed and disarmed, it will be in IDLE, so a new waypoint file read request can be set
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 1}'
# and executed again 
rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 0, request: 2}'
``` 