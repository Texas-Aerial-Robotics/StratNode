#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <deque>
#include <cmath>


using namespace std;


std_msgs::String mode;
geometry_msgs::PoseStamped RoombaPose;
geometry_msgs::PoseStamped waypoint;
geometry_msgs::PoseStamped newpoint;
deque<geometry_msgs::PoseStamped> points;
bool knew = 0;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 newpoint=*msg;
 newpoint.pose.position.z=2.9;
 if (newpoint.pose.position.x != points.front().pose.position.x || newpoint.pose.position.y != points.front().pose.position.y){
    knew = 1;
    if (points.size() <= 8){
     points.push_front(newpoint);
      cout<<"pop front" << endl;
    }
    else {
     points.pop_back();
     points.push_front(newpoint);
     cout<<"pop back"<< endl;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linreg");

  ros::NodeHandle n;
  geometry_msgs::PoseStamped buffer;
  //buffer.pose.position.x = 0;
  //buffer.pose.position.y = 0;
  //buffer.pose.position.z = 2.9;
  //points.push_front(buffer);

  ros::Subscriber sub = n.subscribe("roombaPose", 1, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("mode", 1);
 
  string ss_mode = "SEARCH";
  mode.data = ss_mode.c_str();
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
  double mx = 0, my = 0, sx = 0, sy = 0, st = 0, sxt = 0, syt = 0, sy2 = 0, sx2 = 0;


    if (points.size() >= 8){
       for (int i = 1; i < points.size(); i++){

          double nx = points[i].pose.position.x - points.back().pose.position.x;
          double ny = points[i].pose.position.y - points.back().pose.position.y;
          double nt = (points[i].header.stamp.sec + points[i].header.stamp.nsec * pow(10,-9)) - (points.back().header.stamp.sec + points.back().header.stamp.nsec * pow(10,-9));
          cout <<"nt: " << nt << endl;
          sx = sx + nx;
          sy = sy + ny;
          st = st + nt; 
          sxt = sxt + nx * nt;
          syt = syt + ny * nt;
          sx2 = sx2 + nx * nx;
          sy2 = sy2 + ny * ny;

          //cout<<endl<<"point "<< points[i]<<endl;
       }

       cout<<"delta T in use: "<<st<<endl;
       cout<<"the back x point is "<<points.back().pose.position.x<<endl;

       mx = ((points.size()-1)*sxt-sx*st)/((points.size()-1)*sx2-(sx*sx));
       my = ((points.size()-1)*syt-sy*st)/((points.size()-1)*sy2-(sy*sy));
       
    }
    cout<<"N "<<points.size() - 1<<endl;
    cout<<"sx: "<<sx<<endl;
    cout<<"sxt "<<sxt<<endl;
    cout<<"sx2 "<<sx2<<endl;
    cout<<"dx/dt: "<<mx<<endl;
    cout<<"dy/dt: "<<my<<endl;
    double mxy = sqrt(pow(mx,2) + pow(my,2));

    mx = mx/mxy;
    my = my/mxy;
    cout<<"dx/dt norm: "<<mx<<endl;
    cout<<"dy/dt norm: "<<my<<endl;

    waypoint.pose.position.x = (10 *.33 *my);
    waypoint.pose.position.y = (10 *.33 *mx - 2);
    waypoint.pose.position.z = 2.9; 
    
    if (points.size() >= 8)
    {
       string ss_mode = "GOTO";
       mode.data = ss_mode.c_str();
      while (ros::ok())
      {
        chatter_pub.publish(waypoint);
        mode_pub.publish(mode);
      }
      break;
    }
    mode_pub.publish(mode);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
