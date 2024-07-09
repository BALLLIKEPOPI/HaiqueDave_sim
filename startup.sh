#!/bin/zsh

# gnome-terminal -t "roscore" -x zsh -c "roscore;exec zsh;"
# sleep 2s

gnome-terminal -t "dave" -x zsh -c "source devel/setup.zsh;roslaunch dave_robot_launch haique.launch;exec zsh;"
sleep 5s

# for communication testing
# gnome-terminal -t "communicate" -x zsh -c "source devel/setup.zsh;rosrun communicationtest communicationtest_node ;exec zsh;"
# sleep 1s

gnome-terminal -t "MPC_run" -x zsh -c "source devel/setup.zsh;roslaunch mpc_control mpc_control.launch;exec zsh;"