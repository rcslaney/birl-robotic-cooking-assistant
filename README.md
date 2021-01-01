# Robotic cooking assistant platform

This is the GitHub repository for my masters project at the University of Cambridge in collaboration with the Bio Inspired Robotics Lab lead by Dr. Fumiya Iida. The project is creating a software platform for teaching a robot to cook by demonstration of simple cooking tasks. The idea is the demonstrator moves the robot arm around demonstrating various cooking tasks, this gets recorded and broken down into various cooking primitives (frying, stirring, pouring, etc.) that can then be easily recombined into full recipies. This requires the following tasks:
* Recording of arm trajectories
* Recording of object locations (utensils, pans, etc.)
* An automatic segmentation algorithm to break the demonstration into logical chunks that represent cooking primitives and "moves" between primitives that can be gap filled later on
* A parameterisation algorithm that makes these cooking primitives adaptable to changes in the environment (with stirring for example: spoon and bowl location, bow width, bowl depth, stirring speed, etc.)
* A final program that provides a simple UI to combine the cooking primitives into full recipes.

The project and the current progress is explained more in detail in the presentation [here](https://docs.google.com/presentation/d/1jZk1TQn-gURfQ-hPvTw2JTkabvs3-6zpy5akpkEdTEU/edit?usp=sharing).

It is currently a work in progress, the main application is at [ur5sim/main.py](ur5sim/main.py). It's a TKinter application where all the recording of robot arm trajectories and object locations using AprilTags occurs, as well as visualisation and replay of recorded trajectories. As well as this in the same folder there are some scripts that plot and analyse the recordings.

I am using a version of the AprilTags library modified to allow different tag sizes to be specified for different tag IDs for use in the pose estimation calculation. The modified file is apriltags.py, a copy of which is at [apriltags_modified.py](apriltags_modified.py).

# Used repositories
* [Generic UR5 Controller](https://github.com/kg398/Generic_ur5_controller)
* [UR5Sim](https://bitbucket.org/lucascimeca/ur5sim/src/master/)
* [AprilTag library](https://github.com/duckietown/lib-dt-apriltags)