# Repo containing packages created for the Autonomous Systems course at FHV
## Scripts
Currently only contains scripts for wall-following. Note that both wall_follow.py and wall_follow_complex.py are currently the same. 
The complex one is for trying out other methods of wall-following later.

## Launch files
The current launch files launch a gazebo world and a wall follower. 
The name of the world is the same as for the turtlebot3 launch scripts.
Launch files with "complex" in their names will launch the wall_follower_complex.py after the world.
Note that the "export TURTLEBOT3_MODEL=nameOfModel" command needs to be executed first.
If desired, "export SVGA_VGPU10=0" can be used to change hardware acceleration.