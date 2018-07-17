#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include "transformations_ros/roombaPoses.h"
#include "transformations_ros/roombaPose.h"
#include <sstream>
#include "matplotlibcpp.h"
#include <cmath>
#include <deque>
#include <vector>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>

using namespace std;

int MAX_POINTS = 10;
int marker=0;

geometry_msgs::PoseStamped waypoint;
transformations_ros::roombaPoses roombaPositions;
vector<deque<geometry_msgs::PoseStamped>> decks;
nav_msgs::Odometry current_pose;
std_msgs::Float64 gymOffset;
std::stringstream ss_mode;
std_msgs::String msg;

double dist=90000;
double temp;
int MODE = 0;
int TARGETQ = 100;
struct slope
{
  float mxdt;
  float mydt;
};

void enu_2_gym(nav_msgs::Odometry current_pose_enu)
{
  float GYM_OFFSET = gymOffset.data;
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  float X = x*cos(GYM_OFFSET*deg2rad) - y*sin(GYM_OFFSET*deg2rad);
  float Y = x*sin(GYM_OFFSET*deg2rad) + y*cos(GYM_OFFSET*deg2rad);
  float Z = z;
  current_pose.pose.pose.position.x = X;
  current_pose.pose.pose.position.y = Y;
  current_pose.pose.pose.position.z = Z;


}
void gym_cb(const std_msgs::Float64::ConstPtr& msg)
{
  gymOffset = *msg;
  //ROS_INFO("current heading: %f", current_heading.data);
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  nav_msgs::Odometry current_pose_enu = *msg;
  enu_2_gym(current_pose_enu);
  //ROS_INFO("pose enu x: %f y: %f z: %f", current_pose_enu.pose.pose.position.x, current_pose_enu.pose.pose.position.y, current_pose.pose.pose.position.z);
}
void roomba_cb(const transformations_ros::roombaPoses::ConstPtr& msg)
{
 roombaPositions = *msg;
 double slope1;
 double diffx;
 double diffy;
 double b;

if (roombaPositions.roombaPoses.size() > 0 )
{
    if (decks.size()== 0){

       cout<<"URMOM"<<endl;
       decks.push_back(deque<geometry_msgs::PoseStamped>());
       decks[0].push_front(roombaPositions.roombaPoses[0].roombaPose);
    }

   for (int i=0; i < roombaPositions.roombaPoses.size(); i++){
    dist=9000;
    int rpt = 0;
    int ctr=0;
    double spread1=99;


      for (int j=0; j < decks.size(); j++){


        temp =sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2));
        if(temp<dist){
          dist=temp;
          ctr=j;
          //slope1=diffy/diffx;

        }

      }
      if(decks[ctr].size()>2 && decks[ctr].front().header.stamp.toSec()!=roombaPositions.roombaPoses[i].roombaPose.header.stamp.toSec() ){
           diffx=decks[ctr].front().pose.position.x-decks[ctr][1].pose.position.x;
           cout<<"diffx: "<<diffx<<endl;
           diffy=decks[ctr].front().pose.position.y-decks[ctr][1].pose.position.y;
           cout<<"diffy: "<<diffy<<endl;
           slope1=diffy/diffx;
           cout<<"slope: "<<slope1<<endl;
           b=decks[ctr].front().pose.position.y-slope1*decks[ctr].front().pose.position.x;
           spread1=abs(slope1*roombaPositions.roombaPoses[i].roombaPose.pose.position.x+b-roombaPositions.roombaPoses[i].roombaPose.pose.position.y);
    }
         cout<<"spread calculated as "<<spread1<<endl;
      if ( dist<= 1.5 || (spread1<1 && dist <= 3) ){
            //Found roomba close to deck[j] and adding it to the queue
            if (decks[ctr].front().header.stamp.toSec()!=roombaPositions.roombaPoses[i].roombaPose.header.stamp.toSec()){
            decks[ctr].push_front(roombaPositions.roombaPoses[i].roombaPose);
            }
            //cout << "roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << ctr<< endl;
            //cout<< "Located "<<dist << "away from nearest queue"<<endl;
            if(decks[ctr].size() >= MAX_POINTS){decks[ctr].pop_back();}
          }else{
             //cout<< "Located "<<dist << "away from nearest queue"<<endl;
             if(decks.size()<10){
             decks.push_back(deque<geometry_msgs::PoseStamped>());
             decks[decks.size()-1].push_front(roombaPositions.roombaPoses[i].roombaPose);
             //cout << "New roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << decks.size()-1 << endl;
           }else{

            while (decks[marker].size()>1)
                {

                  decks[marker].pop_back();
                }

             decks[marker].push_front(roombaPositions.roombaPoses[i].roombaPose);

             //cout<< "No queue matches current roomba at FUCKU"<<endl;

           }
               }

    }
  }
}
slope linreg(deque<geometry_msgs::PoseStamped> points)
{
  slope currentSlope;
  double mx = 0, my = 0, sx = 0, sy = 0, st = 0, sxt = 0, syt = 0, sy2 = 0, sx2 = 0, st2 = 0, ix = 0, iy = 0;
  for (int i = 0; i < points.size()-1; i++){

        double nx = points[i].pose.position.x - points.back().pose.position.x;
        double ny = points[i].pose.position.y - points.back().pose.position.y;
        double nt = points[i].header.stamp.toSec() - points.back().header.stamp.toSec();
        sx = sx + nx;
        sy = sy + ny;
        st = st + nt;
        sxt = sxt + nx * nt;
        syt = syt + ny * nt;
        sx2 = sx2 + nx * nx;
        sy2 = sy2 + ny * ny;
        st2 = st2 + nt * nt;
     }
     currentSlope.mxdt = ((points.size()-1)*sxt-sx*st)/((points.size()-1)*st2-(st*st)); // dx/dt
     currentSlope.mydt = ((points.size()-1)*syt-sy*st)/((points.size()-1)*st2-(st*st)); // dy/dt
  return currentSlope;
}
void timeCheck()
{
  ros::Time currentTime = ros::Time::now();
  double currentTime_d = currentTime.toSec();
  double dt=0;
  double temp;
  double dt2;
  for (int i=0; i<decks.size(); i++)
  {
    temp = currentTime_d - decks[i].front().header.stamp.toSec();
    if(temp>dt){
      dt=temp;
      marker=i;
      //cout<<marker<<endl;
     }
  }
}

void target()
{
  int distance = 7;
  ros::Time currentTime = ros::Time::now();
  double currentTime_d = currentTime.toSec();
  // cout << fixed << "Time " << currentTime_d << endl;
  for(int i=0; i<decks.size(); i++)
  {

    if (decks[i].size() > 7 && currentTime_d > 45 && decks[i].front().pose.position.x!=0 && decks[i].front().pose.position.y!=0)
    {
      float dydt;
      float dxdt;
      float sumdydt = 0;
      float sumdxdt = 0;
      float dataPoints = 7;

      for(int j=0; j<dataPoints-1; j++)
      {
        sumdydt = (decks[i][j].pose.position.y - decks[i][j+1].pose.position.y)/(decks[i][j].header.stamp.toSec() - decks[i][j+1].header.stamp.toSec()) + sumdydt;
        sumdxdt = (decks[i][j].pose.position.x - decks[i][j+1].pose.position.x)/(decks[i][j].header.stamp.toSec() - decks[i][j+1].header.stamp.toSec()) + sumdxdt;
      }
      dydt = sumdydt/(dataPoints-1);
      dxdt = sumdxdt/(dataPoints-1);


      float intX = dxdt*(1/dydt)*(0-decks[i].front().pose.position.y) + decks[i].front().pose.position.x;
      cout << "int X " << intX << endl;
      //decide if there is enough data to execute an interaction
      if(decks[i].front().pose.position.y < distance && decks[i].front().pose.position.y > 0 && dydt < -.05 && intX > -1 && intX < 21)  // NEED TO ADD A WAY TO DECIDE WHICH ROOMBA IS BEST TO GO FOR
      {
        waypoint.pose.position.x = decks[i].front().pose.position.x;
        waypoint.pose.position.y = decks[i].front().pose.position.y;
        waypoint.pose.position.z = 1;
        distance = decks[i].front().pose.position.y;
        MODE = 1;
        cout << "set new waypoit x "<< waypoint.pose.position.x << " y " << waypoint.pose.position.y << " z " <<waypoint.pose.position.z << endl;
        TARGETQ = i;
      }
    }
  }



}
int main(int argc, char **argv)
{
  std::vector<double> x(1);
  std::vector<double> y(1);
  ros::init(argc, argv, "stratnode");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<transformations_ros::roombaPoses>("roombaPoses", 1, roomba_cb);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("TARwaypoint", 1);
  ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("setHeading", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("mode", 1);
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber gym_offset_sub = n.subscribe("/gymOffset", 1, gym_cb);


  ros::Rate loop_rate(100);
  int counter=0;
  vector<double> reset_pos_x(3);
  vector<double> reset_pos_y(3);
  reset_pos_x[0]=6.5;
  reset_pos_y[0]=2.5;
  reset_pos_x[1]=10;
  reset_pos_y[1]=2;
  reset_pos_x[2]=13.5;
  reset_pos_y[2]=2.5;

  waypoint.pose.position.x = reset_pos_x[counter%3];
  waypoint.pose.position.y = reset_pos_y[counter%3];
  waypoint.pose.position.z = 1.5;
  counter++;
  cout<<"counter"<<counter<<endl;
  cout<<"index:"<< counter%3 <<endl;
  float heading = 0;
  std_msgs::Float64 current_heading;
  current_heading.data = heading;

  int count = 0;
  float tollorance = .35;
  float mode2SetTime;
  float mode3SetTime;
  float mode4SetTime;
  ss_mode.str("");
  ss_mode.clear();
  ss_mode.str("SEARCH");
  double currentTime;
  float deltaX;
  float deltaY;
  float deltaZ;
  MODE = -1;
 current_heading.data = 45;
  while (ros::ok())
  {
    currentTime = ros::Time::now().toSec();
    cout << fixed << currentTime << endl;
    //check if data has been recieved
    if (decks.size() > 0)
    {
      //Clear old decks
      timeCheck();

      if( MODE == -1)
      {
        current_heading.data = 45;
        heading_pub.publish(current_heading);
        target();
        if (MODE == 1 && decks[TARGETQ].front().pose.position.x > 9 )
        {
          MODE = -1;
        }
      }
      //if in in targeting mode
      if ( MODE == 0)
      {
        target();
        cout << "MODE : " << MODE << endl;

      }
      // if in intercept mode
      if (MODE == 1)
      {
        slope targetDirection;
        targetDirection = linreg(decks[TARGETQ]);
        cout << "slope x " << targetDirection.mxdt << " y " << targetDirection.mydt  << endl;
        double mxy = sqrt(pow(targetDirection.mxdt,2) + pow(targetDirection.mydt,2));


        float dxdt;
        float dydt;
        float sumDPos = 0;
        float dataPoints = 4;
        for(int i=0; i<dataPoints-1; i++)
        {
          dxdt = (decks[TARGETQ][i].pose.position.x - decks[TARGETQ][i+1].pose.position.x)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec());
          dydt = (decks[TARGETQ][i].pose.position.y - decks[TARGETQ][i+1].pose.position.y)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec());
        }
        heading = -atan2(dydt, dxdt)*(180/3.1416) + 90;

        heading = -atan2(targetDirection.mydt/mxy, targetDirection.mxdt/mxy)*(180/3.1416) + 90;
        cout << "heading " << heading << endl;
        current_heading.data = heading;
        heading_pub.publish(current_heading);
        MODE = 2;
        cout << "MODE : " << MODE << endl;
        mode2SetTime = ros::Time::now().toSec();

      }
      if (MODE == 2)
      {

        deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
        deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
        deltaZ = abs(waypoint.pose.position.z - current_pose.pose.pose.position.z);
        //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
        float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
        //cout << dMag << endl;

        if( dMag < tollorance)
        {
          float roombaVelTol = .1;
          float dxdt;
          float dydt;
          float sumDPos = 0;
          float dataPoints = 4;
          float dPos;
          float dTol = .12;
          for(int i=0; i<dataPoints-1; i++)
          {
            dxdt = (decks[TARGETQ][i].pose.position.x - decks[TARGETQ][i+1].pose.position.x)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec());
            dydt = (decks[TARGETQ][i].pose.position.y - decks[TARGETQ][i+1].pose.position.y)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec());
            sumDPos = sqrt( pow(dxdt, 2) + pow(dydt, 2))  + sumDPos;
          }
          dPos = sumDPos/(dataPoints-1);
          //cout << "dPos " << dPos << endl;
          if(abs(dPos) < dTol)
          {
            float dx = current_pose.pose.pose.position.x - decks[TARGETQ].front().pose.position.x;
            float dy = current_pose.pose.pose.position.y - decks[TARGETQ].front().pose.position.y;
            float magdydx = sqrt( pow(dx, 2) + pow(dy, 2));
            waypoint.pose.position.x = decks[TARGETQ].front().pose.position.x + 1*(dx/magdydx);
            waypoint.pose.position.y = decks[TARGETQ].front().pose.position.y + 1*(dy/magdydx);
            waypoint.pose.position.z = .5;
            MODE = 3;
            cout << "MODE : " << MODE << endl;
            mode3SetTime = ros::Time::now().toSec();

          }

        }
        if (currentTime - mode2SetTime > 16 )
          {
            cout << "giving up. Timed out" << endl;
            MODE = 0;
            //clear Q with lost roomba
            while (decks[TARGETQ].size()>1)
            {
              decks[TARGETQ].pop_back();
            }

            waypoint.pose.position.x = reset_pos_x[counter%3];

            waypoint.pose.position.y = reset_pos_y[counter%3];
            waypoint.pose.position.z = 1.5;
            counter++;
            heading = 0;
            cout<<"counter"<<counter<<endl;
            cout<<"index:"<< counter%3 <<endl;
            current_heading.data = heading;
          }
      }
      if(MODE == 3)
      {

        deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
        deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
        deltaZ = abs(waypoint.pose.position.z - current_pose.pose.pose.position.z);
        //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
        float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
        //cout << dMag << endl;

        if( dMag < tollorance)
        {
          ss_mode.str("");
          ss_mode.clear();
          ss_mode.str("LAND");
        }
        if(currentTime - mode3SetTime > 10)
        {
            MODE = 4;
            mode4SetTime = ros::Time::now().toSec();
            waypoint.pose.position.x = reset_pos_x[counter%3];
            waypoint.pose.position.y = reset_pos_y[counter%3];
            waypoint.pose.position.z = 1.5;
            counter++;
            cout<<"counter"<<counter<<endl;
            cout<<"index:"<< counter%3 <<endl;
            heading = 0;
            current_heading.data = heading;
            cout << "MODE : " << MODE << endl;
            ss_mode.str("");
            ss_mode.clear();
            ss_mode.str("TAKEOFF");
        }
      }
      if(MODE == 4)
      {
        if(currentTime - mode4SetTime > 2)
        {
          MODE = 0;
          cout << "MODE : " << MODE << endl;
          ss_mode.str("");
          ss_mode.clear();
          ss_mode.str("SEARCH");

        }
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
    msg.data = ss_mode.str();
    mode_pub.publish(msg);
    heading_pub.publish(current_heading);
    chatter_pub.publish(waypoint);





    //
    //DEBUG PLOTTING
    //
    matplotlibcpp::ion();
    if (roombaPositions.roombaPoses.size())
    {
      matplotlibcpp::xlim(-.5, 19.5);
      matplotlibcpp::ylim(-.5, 19.5);


      // for (int i=0; i < roombaPositions.roombaPoses.size(); i++){
      //    x.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.x);
      //    y.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.y);
      //    matplotlibcpp::plot(x, y, "ro");
      //    matplotlibcpp::pause(0.001);
      //    matplotlibcpp::draw();
      //  }
      string colour[10]= {"bo","go","ro","co","mo","yo","ko","bv","gv","rv"};

      // for (int j=0; j < decks.size(); j++){
      //     //cout<<decks[j][0]<<endl;
      //     //cout<<colour[j]<<endl;

      //     x[0]=decks[j][0].pose.position.x;
      //     y[0]=decks[j][0].pose.position.y;
      //     matplotlibcpp::plot(x, y, colour[j]);
      //     matplotlibcpp::pause(0.0001);
      //     matplotlibcpp::draw();

      // }
    if(TARGETQ!=100){
          x[0]=decks[TARGETQ][0].pose.position.x;
          y[0]=decks[TARGETQ][0].pose.position.y;
          matplotlibcpp::plot(x, y, colour[TARGETQ]);
          matplotlibcpp::pause(0.0001);
          matplotlibcpp::draw();
    }

    }

    ++count;
    // if(count%20==0){
    //  matplotlibcpp::clf();
    // }
  }
  return 0;

}
