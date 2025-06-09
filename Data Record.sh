#!/bin/sh

# Set Session Name
SESSION="FlightRecord"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
	# Start New Session with our name
	tmux new-session -d -s $SESSION

	# Name first Pane and split into multiple
	tmux rename-window -t 0 'Gazebo Rosbag Record'

	tmux select-pane -t 0
	tmux split-window -h   




	# Send commands to pane 0
	tmux send-keys -t 0 'set | grep RECORD_TIME' C-m
	tmux send-keys -t 0 'RECORD_TIME=85' C-m #Add usually 10seconds
	tmux send-keys -t 0 'cd ~/catkin_ws/src/flight_inputs/scripts/Bag_Files' C-m
	tmux send-keys -t 0 'rosbag record --duration=$RECORD_TIME -O subset /mavros/local_position/pose /mavros/setpoint_position/local' C-m 
	tmux send-keys -t 0 'rostopic echo -b subset.bag -p /mavros/local_position/pose > multi08_output.csv' C-m
	tmux send-keys -t 0 'rostopic echo -b subset.bag -p /mavros/setpoint_position/local > multi08_input.csv' C-m
	tmux send-keys -t 0 'rosbag info subset.bag' C-m


	# Send commands to pane 1
	tmux send-keys -t 1 'sleep 5' C-m
	tmux send-keys -t 1 'rostopic hz mavros/local_position/pose' C-m






fi

# Attach Session, on the Main window
tmux attach-session -t $SESSION:0
