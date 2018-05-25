#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <deque>
#include <cmath>


using namespace std;

std_msgs::String msg;
std::stringstream ss_mode;
geometry_msgs::PoseStamped RoombaPose;
geometry_msgs::PoseStamped waypoint;
geometry_msgs::PoseStamped newpoint;
deque<geometry_msgs::PoseStamped> points;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 newpoint=*msg;
 newpoint.pose.position.z=1.5;
 if (newpoint.pose.position.x != points.front().pose.position.x || newpoint.pose.position.y != points.front().pose.position.y){
    if (points.size() <= 5){
     points.push_front(newpoint);
      cout<<"push front" << endl;
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

  ros::Subscriber sub = n.subscribe("roombaPose", 1, chatterCallback);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("mode", 1);
 
  ss_mode << "SEARCH";
  msg.data = ss_mode.str();
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
  ss_mode.str("");
  double mx = 0, my = 0, sx = 0, sy = 0, st = 0, sxt = 0, syt = 0, sy2 = 0, sx2 = 0, st2 = 0, ix = 0, iy = 0;


    if (points.size() >= 5){
       for (int i = 0; i < points.size()-1; i++){

          double nx = points[i].pose.position.x - points.back().pose.position.x;
          double ny = points[i].pose.position.y - points.back().pose.position.y;
          double nt = (points[i].header.stamp.sec + points[i].header.stamp.nsec * pow(10,-9)) - (points.back().header.stamp.sec + points.back().header.stamp.nsec * pow(10,-9));;
          sx = sx + nx;
          sy = sy + ny;
          st = st + nt; 
          sxt = sxt + nx * nt;
          syt = syt + ny * nt;
          sx2 = sx2 + nx * nx;
          sy2 = sy2 + ny * ny;
          st2 = st2 + nt * nt;
          cout <<"nt: " << nt << endl;
          cout <<"sx: " << sx << endl;
          cout <<"sy: " << sy << endl;
          cout <<"st: " << st << endl;
          cout <<"sxt: " << sxt << endl;
          cout <<"syt: " << syt << endl;
          cout <<"sx2: " << sx2 << endl;
          cout <<"sy2: " << sy2 << endl;
          cout <<"nt: " << nt <setHeading< endl << endl;
          //cout<<endl<<"point "<< points[i]<<endl;
       }

       cout<<"delta T in use: "<<st<<endl;
       cout<<"the back x point is "<<points.back().pose.position.x<<endl;
       
      //ix = (sx*st2 - st*sxt)/((points.size()-1)*st2-(st*st));
      //iy = (sy*st2 - sy*syt)/((points.size()-1)*st2-(st*st));
       mx = ((points.size()-1)*sxt-sx*st)/((points.size()-1)*st2-(st*st));
       my = ((points.size()-1)*syt-sy*st)/((points.size()-1)*st2-(st*st));
       
    }
    cout<<"N "<<points.size() - 1<<endl;
    cout<<"x inter: "<<ix<<endl;
    cout<<"y inter: "<<iy<<endl;
    cout<<"dx/dt: "<<mx<<endl;
    cout<<"dy/dt: "<<my<<endl;
    double mxy = sqrt(pow(mx,2) + pow(my,2));

    mx = mx/mxy;
    my = my/mxy;
    cout<<"dx/dt norm: "<<mx<<endl;
    cout<<"dy/dt norm: "<<my<<endl;

    waypoint.pose.position.x = (10 *.33 *mx);
    waypoint.pose.position.y = (10 *.33 *my);
    waypoint.pose.position.z = .5; 
    
    if (points.size() >= 5)
    {
      ss_mode << "GOTO";
      msg.data = ss_mode.str();
      while (ros::ok())
      {
        chatter_pub.publish(waypoint);
        mode_pub.publish(msg);
      }
      break;
    }
    mode_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
