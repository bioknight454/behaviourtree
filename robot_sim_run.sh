#!/usr/bin/env bash

gnome-terminal --tab -- bash -c "ros2 launch linorobot2_gazebo gazebo.launch.py ; echo 'press enter to close'; read -n 1"

gnome-terminal --tab -- bash -c "ros2 run game_pad_sim game_pad_sim; echo 'press enter to close'; read -n 1"

gnome-terminal --tab -- bash -c "ros2 launch linorobot2_navigation navigation.launch.py sim:=true map:=/home/asa/Acel_robot_ws/linorobot2_gazebo/worlds/simulate_abu_wall.world; echo 'press enter to close'; read -n 1"

# gnome-terminal --tab -- bash -c "ros2 run pid_tuning linear_regression; echo 'press enter to close'; read -n 1"

# gnome-terminal --tab -- bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml; echo 'press enter to close'; read -n 1"
# dik: jarak x= 3.5m
#      jarak y = 1.45
#       sudut  = 50.5
