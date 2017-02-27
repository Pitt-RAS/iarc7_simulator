# iarc7_simulator
Simulator that allows testing of algorithms for the IARC Mission 7 competition

## Setup

See [https://github.com/Pitt-RAS/iarc7_common] for install instructions.

## Running the simulator

There's a roslaunch file in `launch/` that has everything you need to run the simulator.  Just run `roslaunch iarc7_simulator morse.launch`, and it will bring everything up.  If it can't find the simulator, make sure you have sourced the setup script from `devel` in the shell you're using.

The roomba control script can be run with `rosrun iarc7_simulator roomba_controller.py` after you've launched the simulator. When you want the roombas to start moving, run the script. The quadcopter can be controlled via the two `uav` topics in ROS, the same ones that `fc_comms` uses.

## Available sensors

The sensors currently available are the Lidar-Lite, an accelerometer, and four cameras.  The appropriate topics are all published on the appropriate topics in ROS.  To adjust the positioning of the sensors on the quadcopter, or to add more sensors, edit `sim/src/sim/builder/Quadcopter.py`.  To adjust the positioning of the robots, edit `sim/default.py`.
