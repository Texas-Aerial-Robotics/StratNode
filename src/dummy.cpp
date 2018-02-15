#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

using namespace std;
geometry_msgs::PoseStamped RoombaPose;

int main(int argc, char **argv){
 ros::init(argc, argv, "Dummy");
 ros::NodeHandle n;

if (argv == "string"){

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("dummy", 1000);
  string message = "home"; 
  }

else{

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("dummy", 1000);
  RoombaPose.pose.position.x = 1;
  RoombaPose.pose.position.y = 1;
  RoombaPose.pose.position.z = 1;

  }

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    chatter_pub.publish(RoombaPose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
 }
  return 0;
}


