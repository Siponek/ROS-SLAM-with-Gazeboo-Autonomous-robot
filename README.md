# Drunk and blind SLAM driving robot
Project during stay at Università di Genova

## Intro
The third assignment of the Research Track 1 course. The goal is to have an robot that uses SLAM algorithm to move around the map in three diffrent modes.


## Get started
### Downloading the packages and installing
The simulator requires ROS. Clone this repository into your ROS workspace's src/ folder.
```
git clone https://github.com/Siponek/ResearchTrack_1080p
```
Additional requirements include:
- final_assignment package,</br>
- the slam_gmapping package,</br>
- ROS navigation stack</br>
```
git clone https://www.github.com/CarmineD8/final_assignment
git clone https://www.github.com/CarmineD8/slam_gmapping
sudo apt-get install ros-noetic-navigation
```
The branch of all these repositores is Noetic.
### Running/walking
Since this is a python package run refresh ros packages:
```
rospack profile
```
To launch the program run the simulation enviroment
```
roslaunch final_assignment simulation_gmapping.launch
roslaunch final_assignment move_base.launch
```
To run the program run the launch file (master_Robot, UI_Robot and teleop_twist_keyboard nodes included)
```
roslaunch assigment_rt_3 launchAssigment.launch
```

## Overview
The robot can be controlled according to three modes:
The package consists of 2 nodes and a lunch files to con these nodes:</br>
- master_Robot            -> as the main control logic node </br>
- UI_Robot                -> as the UI for the user input</br>
- launchAssignment.launch -> as the main launch file    </br>
## Menu

    1: 'Autonomously reach a x,y coordinate inserted by the user',</br>
    2: 'Drive the robot with the keyboard',</br>
    3: 'Drive the robot with assistance to avoid collisions',</br>
    4: 'Exit',</br>
    5: "IT'S TIME TO STOP",</br> - sends reseting singlas on drive modes
    6: "Cancel goal",</br> - cancels autonomous mode from the first option
    
## How it works
**UI_Robot** node sends signals/data to the **master_Robot** on several topics. Remmaped **teleop_twist_keyboard** node is used for steering the robot by the user with manualDrive and assitedDrive modes. **cmd_vel** topic is remmaped to unlock the possibility of assisting the user though **remapped_cmd_vel** topic. 
master_Robot shows the warnings and distances to the nearest obstacles.<br>

The program accepts two parameters andter launch files is specified:</br>
- **assitanceThreshold** - distance from where the assistance starts for option 3</br>
- **timeout** - after what time timeout occurs for option 1</br>





![rosgraph](https://user-images.githubusercontent.com/91413093/155749647-4d459a3d-7212-4051-9f29-3651b6164123.png)

## Flowchart/Pseudocode
### master_Robot
```
  await UI signals/data

  if autonomousDrive
    receive coordinates
    move to specified point

  elseif manualDrive
    use teleop_twist_keyboard to drive the robot

  elseif assistedDrive
    use teleop_twist_keyboard to drive the robot
    scan for obstacles
    change the cmd_vel message to avoid obstacles

  print status
```

### UI_Robot
```
  await input

  while True

    if option == 1
      input coordinates
      publish coordinates to master_Robot
      

    elseif option == 2
      send singal to master_Robot (manualDrive)
      print master_Robot output
      

    elseif option == 3
      send singal to master_Robot (assitedDrive)
      print master_Robot output
      
      
    elif option == 4
      exit
      
    
    elif option == 5
      send reseting signals to reset the drives
    
    elif option == 6
      cancel goal from option 1
  
    else
      print 'Wrong input. Please enter a number ...'

```
