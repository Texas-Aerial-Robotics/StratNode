#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "transformations_ros/roombaPoses.h"
#include "transformations_ros/roombaPose.h"
#include <sstream>
#include "matplotlibcpp.h"
#include <cmath>

using namespace std;
std::vector<double> x, y;

geometry_msgs::PoseStamped waypoint;
transformations_ros::roombaPoses roombaPositions;
void chatterCallback(const transformations_ros::roombaPoses::ConstPtr& msg)
{
 roombaPositions = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stratnode");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<transformations_ros::roombaPoses>("roombaPoses", 1, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    //chatter_pub.publish(waypoint);
    cout << roombaPositions << endl;
    // x.push_back(roombaPositions.roombaPose.pose.position.x);
    // y.push_back(roombaPositions.roombaPose.pose.position.y);
    matplotlibcpp::plot(x, y, "r-");
    // matplotlibcpp::show();
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
