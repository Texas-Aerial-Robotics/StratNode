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
  double mx,my, sx = 0, sy = 0, sxy = 0, sx2 = 0;

    for (i = 0; i <= points.size(); i++){
    
    sx = sx + points(i).pose.position.x;
    sy = sy + points(i).pose.position.y;
    st = st + points(i).header.stamp.secs;
    sxt = sxt + points(i).pose.position.x * points(i).header.stamp.secs;
    syt = sxt + points(i).pose.position.y * points(i).header.stamp.secs;
    sx2 = sx2 * points(i).pose.position.x * points(i).pose.positon.x;
    sy2 = sy2 * points(i).pose.position.y * points(i).pose.position.y;
    }

    
    mx = ((st*sx2 - sx * sxt)/(points.size()*sx2-(sx*sx));
    my = ((st*sy2 - sy * syt)/(points.size()*sx2-(sx*sx));
    
    waypoint.pose.position.x = (10 * mx);
    waypoint.pose.position.y = (10 * my); 
    chatter_pub.publish(waypoint);
    cout<<waypoint<<endl;
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

