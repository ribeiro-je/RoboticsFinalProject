# RoboticsFinalProject
We will be using an MBot Ranger robot with an attached PixyCam for our project. The robot’s main goal will be to drive around looking for a specified object. This will require us to train our model to be able to label the object that we request. Once the object is found the robot will flash its LED lights and sound tune. Additionally, it will drop a pin with its location on the map it has created. 

This will be preformed both in simulation on Gazebo and using a physical Robot.

For mapping and exploration we will be using the technique described in Improved Backtracking Algorithm for Efficient Sensor-based Random Tree Exploration (SRT). The map will be an occupancy map which SRT uses to create its exploration.

### To Run
Have gazebo installed on your machine along with opencv. Two terminal windows are required. Have open open at the top level directory and one open at brain. Do:

```
Top Level
make
./apartment.sh
```

```
Brain
./brain
```

### Repo Structure
- brain
  - src code for control program
- mbot
  - src code uploaded to Mbot Ranger
- model
  - configurations for camera_sensor, led_sensor, tankbot, ultrasonic_sensor
- worlds
  - apartment model world and demo maze world 

# Demo Videos

## Physical Robot

## Gazebo

# Refrences
Random Tree Exploration:
https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees
http://msl.cs.uiuc.edu/rrt/about.html

Paper: “Improved Backtracking Algorithm for Efficient Sensor-based Random Tree Exploration”, 
Haitham El-Hussieny, Samy F. M. Assal and Mohamed Abdellatif Improved Backtracking Algorithm for Efficient Sensor-based Random Tree Exploration

LED:
http://gazebosim.org/tutorials?tut=led_plugin&cat=plugins

Equipment:
https://www.makeblock.com/steam-kits/mbot-ranger
https://pixycam.com/

