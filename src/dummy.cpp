#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <math.h>

using namespace std;

geometry_msgs::PoseStamped testp;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "dummy");
   ros::NodeHandle n;
   int count = 0;
   
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("roombaPose", 1000);
   //create simple dummy cordinates that progress with time for easy troubeshooting
   testp.pose.position.x = count;
   testp.pose.position.y = count;
   testp.pose.position.z = 1.2;


   ros::Rate loop_rate(10);

   while (ros::ok())
   {
      testp.pose.position.x = count;
      testp.pose.position.y = count;
      testp.pose.position.z = 2.9;
      testp.header.stamp.nsec = count;
      chatter_pub.publish(testp);
      cout<<testp<<endl;
      ++count;
      ros::spinOnce();
      loop_rate.sleep();
   }
  return 0;
}
