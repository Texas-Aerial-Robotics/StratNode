roscore &

roslaunch roomba_control linReg.launch
./startsim.sh
cd ~/Controls-Other/Launch/ && roslaunch apm.launch 
roslaunch darknet_ros darknet_ros.launch  || darknet1cam.launch
rosrun transformations_ros simpleTransform  
roslaunch flight_pkg follow.launch
rosrun stratnode lin_reg 
python linRegRoomba.py


#in startsim.sh, "mode guided", 

#this is pretty much exclusively for the big machine (|| are for eric's machine)

#new roomba is started with last command
