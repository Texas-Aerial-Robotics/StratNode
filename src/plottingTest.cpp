#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "transformations_ros/roombaPoses.h"
#include "transformations_ros/roombaPose.h"
#include <sstream>
#include "matplotlibcpp.h"
#include <cmath>
#include <string>
#include <map>
#include <vector>

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
  std::map<std::string, std::string> keywords;
  keywords["color"] = "red";
  keywords["marker"] = "o";
  keywords["linestyle"] = "none";
  while (ros::ok())
  {
    //chatter_pub.publish(waypoint);
    cout << roombaPositions << endl;
    ros::spinOnce();
    loop_rate.sleep();
    matplotlibcpp::ion();
    if (roombaPositions.roombaPoses.size())
    {
      double alpha = 1;
      for (int i = 0; i < roombaPositions.roombaPoses.size(); ++i)
      {
        x.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.x);
        y.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.y);
      }

      matplotlibcpp::clf();
      matplotlibcpp::xlim(-10, 10);
      matplotlibcpp::ylim(-10, 10);
      std::vector<double> stagingx(1);
      std::vector<double> stagingy(1);
      for (int i = x.size(); i > x.size()-160 && i > 0; --i)
      {
        stagingx[0] = x[i];
        stagingy[0] = y[i];
        matplotlibcpp::plot(stagingx, stagingy, keywords, alpha);
        if (alpha > 0.1)
        {
          alpha -= 0.05;
        }
      }
      matplotlibcpp::draw();
      // x.erase(x.begin());
      // y.erase(y.begin());
      matplotlibcpp::pause(0.001);
    }
    ++count;
  }
  return 0;
}
