#!/usr/bin/env bash

# (trap 'kill 0' SIGINT; konsole  -e roslaunch gmapping display2.launch & rosrun map_server map_server rospack find
# gmapping map.yaml & roslaunch gmapping amcl.launch & roslaunch gmapping move_base.launch & roslaunch assigment_rt_3 launchAssigment.launch)

konsole -e roslaunch gmapping display2.launch & fp
wait
konsole -e rosrun map_server map_server $(rospack find gmapping)/map.yaml &

konsole -e roslaunch gmapping amcl.launch &

konsole -e roslaunch gmapping move_base.launch &

konsole -e roslaunch assigment_rt_3 launchAssigment.launch