# Repo containing packages created for the Autonomous Systems course at FHV
# rupp_turtlebot3
Contains nodes/worlds/launch files etc. created for the turtlebot3-exercises.
## Scripts
Contains all python scripts done for the exercises.
### go_to_nearest
Script for heading towards nearest object that robot can detect with his LIDAR sensor.
### image_show
Shows image captured by the robot's camera.
Needs robot model with camera (e.g. waffle) to work.
Shows camera image from robot in separate window (two windows, one for rgb and one for greyscale image).
When image window is closed, new image is window with camera image is created.
Also, when turtlebot is moved, image will be updated with camera feed (looks almost like video-feed).
Killing the image node will also close all image windows.
### tcp_client 
Simple node for a tcp client.
Will publish a short string to a tcp server everytime the callback for the scan-subscriber is called.
Requirest tcp sercver to be running so it can connect.
### tcp_server
Simple node for a tcp server.
Will open and listen to a socket at the same port/address as the client (currently hardcoded).
When receiving a message, will print this message to the screen and then send a string response confirming that a message was received.
Currently completely independen from any turtlebot functionality (can be changed if desired).
### wall_follow_complex
A more complex variant of the wall_follow node, using a combination of bang bang and proportional controllers.
Will always try to put the wall to the left of the robot.
### wall_follow
A very simple wall follower that only uses 10 beams of the lidar sensor.
Only really works with very simple stages.

## Launch files
Contains the launch files for the exercises.
It is recommended to use roslaunch in combination with these launch files instead of starting the nodes one by one (though it is still possible).
### collab
Launches an empty world, places two turtlebots in it and then launches a tcp server and a tcp client for each robot.
Will print the messages the server and the client exchange to the screen.
### complex_wall_follow_stage_1
Uses the existing stage_1 launch file (from turtlebot3) to load stage 1 and spawn a turtlebot.
Also launches the complex wall follower.
The wall follower will print the angles currently used for twist.angular.z to the screen.
Note: sometimes, the scan topic stops receiving messages (even though gazebo continues running).
Should the turtlebot stop wall following, check whether there are still new prints to the screen or not.
### complex_wall_follow_world
Same as "complex_wall_follow_stage_1", only using the world launch file instead of the stage 1 launch file.
### goto_simple
Loads the very simple world (only one object in close proximity to the robot), spawns a robot and launches a go_to_nearest node.
Robot should head straight for the object.
### goto_spheres
Similar to goto_simple, only the loaded world contains several spheres in various proximities to the robot.
The closest one should be to the left of the initial robot position if the world is not changed.
### goto_world
Also similar to goto_simple, only for this launch file, the world launch file from the turtlebot3 package is used.
The closest object should be the wall behind the robot's initial position.
### modified_world
Loads the modified world provided for the camera exercise and spawns a robot in it.
Used in the show_image launch file.
### show_image
Uses the modified_world launch file to load the modified world and spawn a robot.
Also launches an image_show node.
Images ill be opened in new extra windows.
Note: to control the turtlebot, use this launch file, then execute "rosrun turtlebot3_teleop turtlebot3_teleop_key".
### simple_world
Loads the world with only one object in it created for the go_to_nearest node and spawns a robot in it.
Used in other launch files.
### spheres_world
Loads the world with several spheres in it created for the go_to_nearest node and spawns a robot in it.
Used in other launch files.
### wall_follow_stage_1.launch
Uses the existing stage_1 launch file (from turtlebot3) to load stage 1 and spawn a turtlebot.
Also launches the simple wall follower.
Will print the shorthand for the sensor with the currently smallest value to the console.
Shorthands are in form: RF - right front, RDF - right diagonal front, R - right, RDB - right diagonal back, RB - right back (L instead of R for left).
Note: sometimes, the scan topic stops receiving messages (even though gazebo continues running).
Should the turtlebot stop wall following, check whether there are still new prints to the screen or not.
### wall_follow_world
Same as "wall_follow_stage_1", only using the world launch file instead of the stage 1 launch file.
## Worlds
Contains custom worlds created for the sake of the exercises.
The worlds already provided by the turtlebot3 package can be found under "turtlebot3_simulations/turtlebot3_gazebo/worlds".
### goto_nearest_simple
A world only containing one object close to the origin (x,y,z = 0).
### goto_nearest_spheres
A world containing several spheres in various proximities to the origin.
### turtlebot3_stage1_modified
World provided for the camera exercise.
Is a modification of stage 1 with several cylinders of different colors.