# pruning-experiments

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
