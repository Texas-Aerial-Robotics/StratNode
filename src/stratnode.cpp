#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"

#include <sstream>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    gazebo_msgs::ModelStates newState = (gazebo_msgs::ModelStates)(*msg);

    ROS_INFO("New Model State: %s\nPosition: %f, %f, %f\nTwist: %f, %f, %f; %f, %f, %f\n\n",
     newState.name[0].c_str(),
     newState.pose[0].position.x,
     newState.pose[0].position.y,
     newState.pose[0].position.z,
     newState.twist[0].linear.x,
     newState.twist[0].linear.y,
     newState.twist[0].linear.z,
     newState.twist[0].angular.x,
     newState.twist[0].angular.y,
     newState.twist[0].angular.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "stratnode");

  ros::NodeHandle nh;


  ros::Subscriber state_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, modelStatesCb);

 

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

