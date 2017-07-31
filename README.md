# iarc7\_simulator
Simulator that allows testing of algorithms for the IARC Mission 7 competition

For other IARC teams finding this repository, we've made it public intentionally. We would love for you to take a look around and send us any questions you have (or any bugfixes you develop). This code is open source and free to use under the GPL, but we do ask that other IARC teams using this code or ideas taken from it cite the Pitt Robotics and Automation Society and do not present the work or the ideas contained within it as their own.

## Setup

See https://github.com/Pitt-RAS/iarc7_common for install instructions.

## Running the simulator

There's a roslaunch file in `launch/` that has everything you need to run the simulator.  Just run `roslaunch iarc7_simulator morse.launch`, and it will bring everything up.  If it can't find the simulator, make sure you have sourced the setup script from `devel` in the shell you're using.

The roomba control script can be run with `rosrun iarc7_simulator roomba_controller.py` after you've launched the simulator. When you want the roombas to start moving, run the script. The quadcopter can be controlled via the two `uav` topics in ROS, the same ones that `fc_comms` uses.

## Available sensors

The sensors currently available are the Lidar-Lite, VL53L0X, an accelerometer, five cameras, foot switches, and RPLIDAR A2.  The appropriate measurements are all published on the appropriate topics in ROS.  To adjust the positioning of the sensors on the quadcopter, or to add more sensors, edit `sim/src/sim/builder/Quadcopter.py`.  To adjust the positioning of the robots, edit `sim/default.py`.
