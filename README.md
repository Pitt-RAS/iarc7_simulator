# iarc7-simulator
Simulator that allows testing of algorithms for the IARC Mission 7 competition

## Setup

First, you want to clone the repo and put it in the `src` directory of your catkin workspace.  You want to rename the folder to `iarc7_simulator` too. You should then be able to run `catkin_make` and have everything compile. You then need to run the following:

    cd iarc7_simulator
    morse import sim

## Running the simulator

There's a roslaunch file in `launch/` that has everything you need to run the simulator.  Just run `roslaunch iarc7_simulator morse.launch`, and it will bring everything up.  If it can't find the simulator, make sure you have sourced the setup script from `devel` in the shell you're using.

The roomba control script can be run with `python3 sim_client.py` from the `scripts` directory. When you want the roombas to start moving, run the script. The quadcopter can be controlled via the two `uav` topics in ROS, the same ones that `fc_comms` uses.

## Available sensors

The sensors currently available are the Lidar-Lite, an accelerometer, and four cameras.  The appropriate topics are all published on the appropriate topics in ROS.  To adjust the positioning of the sensors on the quadcopter, or to add more sensors, edit `sim/src/sim/builder/Quadcopter.py`.  To adjust the positioning of the robots, edit `sim/default.py`.
