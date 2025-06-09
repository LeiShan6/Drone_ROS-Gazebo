#!/bin/sh

# Set Session Name
SESSION="FlightBoot"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Only create tmux session if it doesn't already exist
if [ "$SESSIONEXISTS" = "" ]
then
	# Start New Session with our name
	tmux new-session -d -s $SESSION

	# Name first Pane and split into multiple
	tmux rename-window -t 0 'Gazebo Tmux Startup'

	tmux select-pane -t 0
	tmux split-window -h
	tmux split-window -v   
	tmux select-pane -t 0
	tmux split-window -v



	# Send commands to pane 0
	tmux send-keys -t 0 'sleep 8' C-m
	tmux send-keys -t 0 'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"' C-m 


	# Send commands to pane 1
	tmux send-keys -t 1 'sleep 20' C-m
	tmux send-keys -t 1 'cd ~/src/Firmware' C-m
	tmux send-keys -t 1 'source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default' C-m
	tmux send-keys -t 1 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)' C-m
	tmux send-keys -t 1 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo' C-m
	tmux send-keys -t 1 'roslaunch px4 posix_sitl.launch' C-m



	# Send commands to pane 2
	tmux send-keys -t 2 'cd ~/catkin_ws/src' C-m 'src' C-m
	tmux send-keys -t 2 'sleep 45' C-m
	tmux send-keys -t 2 'rosrun flight_inputs offb_test_node.py' C-m


	# Send commands to pane 3
	tmux send-keys -t 3 'cd' C-m 
	tmux send-keys -t 3 'roscore' C-m


fi

# Attach Session, on the Main window
tmux attach-session -t $SESSION:0
