roslaunch gazebo_ros_sim roombaSim.launch
./startsim.sh 
roslaunch apm.launch
roslaunch darknet_ros darknet_ros.launch 
roslaunch camera_signalman camera_signalman_nodelet.launch
roslaunch transformations_ros transformations.launch
rosrun stratnode stratnode

in apm terminal:
mode guided
arm throttle
takeoff 2
#these instructions are for Mark's comp, as of 6/12/18
