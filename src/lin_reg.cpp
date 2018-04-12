#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

using namespace std;

geometry_msgs::PoseStamped RoombaPose;
geometry_msgs::PoseStamped waypoint;
geometry_msgs::PoseStamped newpoint;


void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 newpoint=*msg;
 waypoint.pose.position.z=1.5;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linreg");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("roombaPose", 1, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    chatter_pub.publish(waypoint);
    cout<<waypoint<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

