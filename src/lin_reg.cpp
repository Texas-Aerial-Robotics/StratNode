#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <deque>


using namespace std;

geometry_msgs::PoseStamped RoombaPose;
geometry_msgs::PoseStamped waypoint;
geometry_msgs::PoseStamped newpoint;
deque<geometry_msgs::PoseStamped> points;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 newpoint=*msg;
 newpoint.pose.position.z=1.5;
 if (points.size() <= 5){
  points.push_front(newpoint);
 }
 else {
  points.pop_back();
  points.push_front(newpoint);
 }
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
  double m, sx = 0, sy = 0, sxy = 0, sx2 = 0;

    for (i = 0; i <= points.size(); i++){
    sx = sx + points(i).pose.position.x;
    sy = sy + points(i).pose.position.y;
    sxy = sxy + points(i).pose.position.x * points(i).pose.position.y;
    sx2 = sx2 * points(i).pose.position.x * points(i).pose.positon.x;
    }
    
    m = ((sy*sx2 - sx * sxy)/(points.size()*sx2-(sx*sx));
    
    
    waypoint = points.front().pose 
    chatter_pub.publish(waypoint);
    cout<<waypoint<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

