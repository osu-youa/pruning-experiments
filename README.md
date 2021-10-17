# pruning-experiments

Code used to run the pruning experiments "Precision fruit tree pruning using a learned hybrid vision/interaction controller" on the ROS side. The non-ROS neural network code used to train the system can be found in this repository: https://github.com/osu-youa/pruning-pybullet 

The main interface used to control the interface is in scripts/run_pruning_experiment.py. It allows you to save poses and run the different controllers. Note that due to the neural network being run on a non-ROS computer, you will have to plug the Realsense into the ROS computer to access the point cloud data, and then plug it into the neural network computer afterwards for running the experiment.

The typical flow of an experiment is:

1. Freedrive the arm to a desired viewpoint of the setup.
2. Select "Save pose from camera" and obtain a target via the point cloud from the camera (you will click on the point in RViz to select it).
3. Load the pose and run the 4 controllers.

## Controller Nodes

__force_filter.py__ subscribes to the /wrench topic and outputs a filtered WrenchStamped to /wrench_filtered. Right now it averages 51 points, and only does the relevant values for the controller: force.y, force.z, and torque.x. 

__admit_ctlr.py__ subscribes to /wrench_filtered and publishes velocity commands to /vel_command. Stop conditions are also included in this node. Currently this activates servoing (with service call to /servo_activate) at the beginning, and stops servoing at the stop condition (with a service call to /servo_stop). 

__contact_watcher.py__ subscribes to /wrench_filtered. When it senses contact, it makes a service call to "delcare_contact" (with message MadeContact.srv), at which point it no longer looks for contact. Watching can be reactivated with a service call to "reset_contact" with the argument MadeContact(True), which is counterintuitive and on my list of things to change. 

## Rosbag Data

There are several rosbag files. They contain /wrench data from experiments. Each are a minute long. 

__wrench_unattended.bag__ and __wrench_with_pushes.bag__ were made before the controller was implemented. The former is left alone for a minute. The latter has some "pushes" made by Hannah pushing on the proxy cutter.

__H_trial1.bag__ and __H_trial2.bag__ are a minute of data with the controller implemented. Trial 1 sets very quickly, but Trial 2 oscillates and never declares the branch successfully set. 

## Launch

__ctlr_launch.launch__ is a work in progress. Currently it launches __force_filter__, __admit_ctlr__, and __contact_server__. 
