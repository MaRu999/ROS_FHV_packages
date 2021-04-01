# Repo containing packages created for the Autonomous Systems course at FHV
# rupp_turtlebot3
Contains nodes/worlds/launch files etc. created for the turtlebot3-exercises.
## Scripts
Contains all python scripts done for the exercises.
### go_to_nearest.py
Script for heading towards nearest object that robot can detect with his LIDAR sensor.
### image_show.py
Shows image captured by the robot's camera.
Needs robot model with camera (e.g. waffle) to work.
Shows camera image from robot in separate window (two windows, one for rgb and one for greyscale image).
When image window is closed, new image is window with camera image is created.
Also, when turtlebot is moved, image will be updated with camera feed (looks almost like video-feed).
Killing the image node will also close all image windows.
### tcp_client.py 
Simple node for a tcp client.
Will publish a short string to a tcp server everytime the callback for the scan-subscriber is called.
Requirest tcp sercver to be running so it can connect.
### tcp_server.py
Simple node for a tcp server.
Will open and listen to a socket at the same port/address as the client (currently hardcoded).
When receiving a message, will print this message to the screen and then send a string response confirming that a message was received.
Currently completely independen from any turtlebot functionality (can be changed if desired).
### wall_follow_complex.py
A more complex variant of the wall_follow node, using a combination of bang bang and proportional controllers.
Will always try to put the wall to the left of the robot.
### wall_follow.py
A very simple wall follower that only uses 10 beams of the lidar sensor.
Only really works with very simple stages.

## Launch files
Contains the launch files for the exercises.
It is recommended to use roslaunch in combination with these launch files instead of starting the nodes one by one (though it is still possible).
### collab.launch
Launches an empty world, places two turtlebots in it and then launches a tcp server and a tcp client for each robot.
Will print the messages the server and the client exchange to the screen.
### complex_wall_follow_stage_1.launch
Uses the existing stage_1 launch file (from turtlebot3) to load stage 1 and spawn a turtlebot.
Also launches the complex wall follower.
The wall follower will print the angles currently used for twist.angular.z to the screen.
Note: sometimes, the scan topic stops receiving messages (even though gazebo continues running).
Should the turtlebot stop wall following, check whether there are still new prints to the screen or not.
### complex_wall_follow_world.launch
Same as "complex_wall_follow_stage_1", only using the world launch file instead of the stage 1 launch file.
### goto_simple.launch
Loads the very simple world (only one object in close proximity to the robot), spawns a robot and launches a go_to_nearest node.
Robot should head straight for the object.
### goto_spheres.launch
Similar to goto_simple, only the loaded world contains several spheres in various proximities to the robot.
The closest one should be to the left of the initial robot position if the world is not changed.
### goto_world.launch
Also similar to goto_simple, only for this launch file, the world launch file from the turtlebot3 package is used.
The closest object should be the wall behind the robot's initial position.
### modified_world.launch
Loads the modified world provided for the camera exercise and spawns a robot in it.
Used in the show_image launch file.
### show_image.launch
Uses the modified_world launch file to load the modified world and spawn a robot.
Also launches an image_show node.
Images ill be opened in new extra windows.
Note: to control the turtlebot, use this launch file, then execute "rosrun turtlebot3_teleop turtlebot3_teleop_key".
### simple_world.launch
Loads the world with only one object in it created for the go_to_nearest node and spawns a robot in it.
Used in other launch files.
### spheres_world.launch
Loads the world with several spheres in it created for the go_to_nearest node and spawns a robot in it.
Used in other launch files.
### wall_follow_stage_1.launch
Uses the existing stage_1 launch file (from turtlebot3) to load stage 1 and spawn a turtlebot.
Also launches the simple wall follower.
Will print the shorthand for the sensor with the currently smallest value to the console.
Shorthands are in form: RF - right front, RDF - right diagonal front, R - right, RDB - right diagonal back, RB - right back (L instead of R for left).
Note: sometimes, the scan topic stops receiving messages (even though gazebo continues running).
Should the turtlebot stop wall following, check whether there are still new prints to the screen or not.
### wall_follow_world.launch
Same as "wall_follow_stage_1", only using the world launch file instead of the stage 1 launch file.
## Worlds
Contains custom worlds created for the sake of the exercises.
The worlds already provided by the turtlebot3 package can be found under "turtlebot3_simulations/turtlebot3_gazebo/worlds".
### goto_nearest_simple.world
A world only containing one object close to the origin (x,y,z = 0).
### goto_nearest_spheres.world
A world containing several spheres in various proximities to the origin.
### turtlebot3_stage1_modified.world
World provided for the camera exercise.
Is a modification of stage 1 with several cylinders of different colors.

# package_tutorial_rupp
Contains a package with simple nodes for Learning the build system.

## Note regardin topic names
The default for all topics is "/turtle1/...", e.g. "turtle1/cmd_vel" for controlling the turtlebot. This is because spawning a turtlesim_node without any remapping will call the topics exactly that. When remapping the turtlesim_node, the respective scripts need to be remapped too in order to function correctly. A concrete example:
  - Starting turtlebot: rosrun turtlesim turtlesim_node /turtle1:=/myturtle
  - Starting controller: rosrun package_tutorial_rupp config_publish.py /turtle1/cmd_vel:=/myturtle/cmd_vel

## Messages
 
### Aggregate 
Custom message for aggregating information from turtlebot. 
Resuses the existing message-types and collects them in one message:
  - Twist
  - Color
  - Pose

## Services 
### Set_Log_time
Service for setting the sleep time of the logging subscriber, meaning the rate with which information is logged. E.g. a value of 5 means a log entry every 5 seconds. Takes a float64 as input, which is the time to sleep. Returns a string confirming the success of setting the time.

### Stop_control
Service for enabling/disabling the node that controls a turtlebot. Takes ans int64 as input. When called with an argument of 0, the control will be disabled. Any other integer will enable it again. Will return a string confirming the success of the action.

## Config files
### movement
Contains the information used by the publisher that controls the turtlebot. Uses a Twist format: x, y, z can be set for linear and angular. Is used to set the paramters on the parameter server, use with a launch file (e.g. only_params.launch).

### node_names
Names for the control, aggregator and logging nodes. Note that using the launch file to launch the nodes will use the names specified there instead.

### rates
Sets the aggregation, logging and control rates (in seconds for the logging, in Hertz for the other two).

## Launch files
### params
Sets all parameters used in the custom package. Does not run any "persistent" nodes, so can be set to set the parameters on the server at any time. Uses the files in the config folder to set those parameters.

### full_launch
Sets the parameters (using the) launch file above) and then starts one turtlesim-node, the controller moving the turtlebot, the aggregator and the logger.

## Scripts
### aggregate_subscriber_custom_msg
Subscriber to the topics used by the turtlebot (cmd_vel, color, pose). Aggregates the information from said topics into one message using the custom Aggregate.msg type. Publishes the aggregated information to the "rupp/state" topic. (Subscriber and Publisher)

### config_publish
Publishes the movement information specified in movement.yaml (needs to be on the parameter server, use launch file for this) to the turtlesim topic cmd_vel. Also offers a service to enable/disable the movement over the "rupp/stop_control" service server. (Publisher and Service Server).

### control_stop_client
Service client for enabling/disabling the config_publish node. Takes one integer argument. 0 will disable control, any other integer will reenable it. (Service Client)

### log_state
Subscribes to the "rupp/state" topic and logs the information published on there to a log-file. Uses the rospy_message_converter to convert the custom aggregate message into a json file, which is then written to the "bot_state.log" file. Said file will currently end up in the package (workspace/src/package_tutorial_rupp). Also offers a service server for changing the sleep time between appends to the log file (meaning the rate at which the information is logged). (Subscriber, Service Server)

### set_log_time_client
Service client for setting the intervals between the logs of the log_state node. Takes one float argument and sets the intervals between log-appends to that many seconds. (Service client).

## Added dependencies
For dependencies added after the package was created with catkin.
### rospy_message_converter
Package for converting ROS-messages to JSON and vice versa. Was incorporated by cloning the github master-branch into the workspace and then executing catkin_make. Used in the aggregate_subscriber_custom_msg node.