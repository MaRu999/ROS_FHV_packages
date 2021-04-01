# Package for learning the ROS build system
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