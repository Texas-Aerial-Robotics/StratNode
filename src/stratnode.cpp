#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

using namespace std;

geometry_msgs::PoseStamped RoombaPose;
geometry_msgs::PoseStamped waypoint;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 waypoint.pose.position.x=msg.pose.position.x;
 waypoint.pose.position.y=msg.pose.position.y;
 waypoint.pose.position.z=1.5;
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stratnode");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("roombaPose", 1, chatterCallback);

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

