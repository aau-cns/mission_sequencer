Command to set filepath to test_trajectory.csv:

rosparam set /autonomy/missions/mission_1/filepaths ["/home/chriboehm/workspaces/mission_ws/src/amaze_mission_sequencer/trajectories/test_trajectory.csv"]

Command for requests:

rostopic pub -1 /autonomy/request amaze_mission_sequencer/request '{id: 1, request: #}'
