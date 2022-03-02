# Obstacle-Avoidance-using-Bug2-algorithm
Implemented Bug2 algorithm to avoid obstacles in the map to move the robot from start location to a goal location for a given map.


Steps to follow:

    Create package inside /catkin_ws/src/

catkin_create_pkg bug2 std_msgs geometry_msgs rospy roscpp

    Copy files inside package and follow:

cd ~/catkin_ws
catkin_make
source ~/.bashrc

    Grant execution permission to the script or .py file

chmod +x <.py file directory>

    Run program by roslaunch:

roslaunch bug2 bug2.launch
