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

    int count =  newState.name.size();

    for(int i=1; i<count; i++){
        ROS_INFO("New Model State: %s\nPosition: %f, %f, %f\nTwist: %f, %f, %f; %f, %f, %f\n\n",
             newState.name[i].c_str(),
             newState.pose[i].position.x,
             newState.pose[i].position.y,
             newState.pose[i].position.z,
             newState.twist[i].linear.x,
             newState.twist[i].linear.y,
             newState.twist[i].linear.z,
             newState.twist[i].angular.x,
             newState.twist[i].angular.y,
             newState.twist[i].angular.z);

    }

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

