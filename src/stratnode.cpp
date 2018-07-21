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

//check to see if the message has any detections
if (roombaPositions.roombaPoses.size() > 0 )
{
    //if there are no decks, make one
    if (decks.size()== 0){

       cout<<"URMOM"<<endl;
       decks.push_back(deque<geometry_msgs::PoseStamped>());
       decks[0].push_front(roombaPositions.roombaPoses[0].roombaPose);
    }

    //cycle through detections
    for (int i=0; i < roombaPositions.roombaPoses.size(); i++){

      //see which deck is closest to detection
      double dist=9000;
      int rpt = 0;
      int ctr=0;
      for (int j=0; j < decks.size(); j++){
        temp =sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2));
        //make sure Q isn't dead
        if(temp<dist && roombaPositions.roombaPoses[i].roombaPose.header.stamp.toSec() - decks[j].front().header.stamp.toSec() < 6){
          dist=temp;
          ctr=j;
          //slope1=diffy/diffx;
        }
      }

      //calculate if the detection lies along the predicted path
      double spread1=99;
      diffx=0;
      diffy=0;
      if(decks[ctr].size()>2 && decks[ctr].front().header.stamp.toSec()!=roombaPositions.roombaPoses[i].roombaPose.header.stamp.toSec() )
      {   for(int z = 0; z<decks[ctr].size()-1;z++){
            diffx=decks[ctr][z].pose.position.x-decks[ctr][z+1].pose.position.x+diffx;
            //cout<<"diffx: "<<diffx<<endl;
            diffy=decks[ctr][z].pose.position.y-decks[ctr][z+1].pose.position.y+diffy;
            //cout<<"diffy: "<<diffy<<endl;
          }
         slope1=diffy/diffx;
         cout<<"slope: "<<slope1<<endl;
         b=decks[ctr].front().pose.position.y-slope1*decks[ctr].front().pose.position.x;
         spread1=abs(slope1*roombaPositions.roombaPoses[i].roombaPose.pose.position.x+b-roombaPositions.roombaPoses[i].roombaPose.pose.position.y);
         cout<< "spread " << spread1 <<endl;
      }
         // /cout<<"spread calculated as "<<spread1<<endl;
      if ( dist<= 1 || (spread1<0.5 && dist <= 1.7) )
      {
        //Found roomba close to deck[j] and adding it to the queue
        //make sure message is a new message
        if (decks[ctr].front().header.stamp.toSec()!=roombaPositions.roombaPoses[i].roombaPose.header.stamp.toSec()){
          decks[ctr].push_front(roombaPositions.roombaPoses[i].roombaPose);
        }
        //cout << "roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << ctr<< endl;
        //cout<< "Located "<<dist << "away from nearest queue"<<endl;
        if(decks[ctr].size() >= MAX_POINTS)
        {
          decks[ctr].pop_back();
        }
      }else{
         //cout<< "Located "<<dist << "away from nearest queue"<<endl;

        //if all the decks have not been made make a new deck
        if(decks.size()<10){
        decks.push_back(deque<geometry_msgs::PoseStamped>());
        decks[decks.size()-1].push_front(roombaPositions.roombaPoses[i].roombaPose);
        //cout << "New roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << decks.size()-1 << endl;
        }else{

        //slaughter the oldest Q and start the new Q in it's place
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
void timeCheck()
{
  //find the oldest Q and mark it for slaughter
  ros::Time currentTime = ros::Time::now();
  double currentTime_d = currentTime.toSec();
  double dt=0;
  double temp;
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
  int distance = 9;
  for(int i=0; i<decks.size(); i++)
  {

    //check if there is 7 points in the Q
    if (decks[i].size() > 6 && (MODE == 0 || MODE ==-1))
    {
      float dydt;
      float dxdt;
      float sumdydt = 0;
      float sumdxdt = 0;
      float dataPoints = 7;

      //take numeric derivative
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
        waypoint.pose.position.z = .85;
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

  ros::Subscriber sub = n.subscribe<transformations_ros::roombaPoses>("roombaPoses", 5, roomba_cb);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("TARwaypoint", 1);
  ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("setHeading", 1);
  ros::Publisher mode_pub = n.advertise<std_msgs::String>("mode", 1);
  ros::Subscriber currentPos = n.subscribe<nav_msgs::Odometry>("mavros/global_position/local", 10, pose_cb);
  ros::Subscriber gym_offset_sub = n.subscribe("/gymOffset", 1, gym_cb);


  ros::Rate loop_rate(15);
  int counter=0;
  vector<double> reset_pos_x(3);
  vector<double> reset_pos_y(3);
  reset_pos_x[0]=6.5;
  reset_pos_y[0]=3.5;
  reset_pos_x[1]=10;
  reset_pos_y[1]=2;
  reset_pos_x[2]=13.5;
  reset_pos_y[2]=4.5;

  waypoint.pose.position.x = reset_pos_x[counter%3];
  waypoint.pose.position.y = reset_pos_y[counter%3];
  waypoint.pose.position.z = 1.5;
  counter++;

  float heading = 0;
  std_msgs::Float64 current_heading;

  int count = 0;
  float tollorance = .35;
  double lastmode0SetTime;
  double mode2SetTime;
  double mode3SetTime;
  double mode4SetTime;
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
    ROS_INFO("MODE %d ", MODE);
    currentTime = ros::Time::now().toSec();
    // cout << fixed << currentTime << endl;
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
        if (MODE == 1 && decks[TARGETQ].front().pose.position.x > 11 )
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
        float dxdt;
        float dydt;
        float sumdxdt = 0;
        float sumdydt = 0;
        float sumDPos = 0;
        float dataPoints = 7;
        float dPos;
        for(int i=0; i<dataPoints-1; i++)
        {
          sumdxdt = (decks[TARGETQ][i].pose.position.x - decks[TARGETQ][i+1].pose.position.x)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec()) + sumdxdt;
          sumdydt = (decks[TARGETQ][i].pose.position.y - decks[TARGETQ][i+1].pose.position.y)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec()) + sumdydt;

        }
        dydt = sumdydt/(dataPoints-1);
        dxdt = sumdxdt/(dataPoints-1);

        heading = -atan2(dydt, dxdt)*(180/3.1416) + 90;

        cout << "heading " << heading << endl;
        current_heading.data = heading;
        heading_pub.publish(current_heading);
        MODE = 2;
        cout << "MODE : " << MODE << endl;
        mode2SetTime = ros::Time::now().toSec();

      }
      if (MODE == 2)
      {

        //numeric derivative
        float dxdt;
        float dydt;
        float sumdxdt = 0;
        float sumdydt = 0;
        float sumDPos = 0;
        float dataPoints = 4;
        float dPos;
        float dTol = .05;
        for(int i=0; i<dataPoints-1; i++)
        {
          sumdxdt = (decks[TARGETQ][i].pose.position.x - decks[TARGETQ][i+1].pose.position.x)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec()) + sumdxdt;
          sumdydt = (decks[TARGETQ][i].pose.position.y - decks[TARGETQ][i+1].pose.position.y)/(decks[TARGETQ][i].header.stamp.toSec() - decks[TARGETQ][i+1].header.stamp.toSec()) + sumdydt;

        }
        dydt = sumdydt/(dataPoints-1);
        dxdt = sumdxdt/(dataPoints-1);
        sumDPos = sqrt( pow(sumdxdt, 2) + pow(sumdydt, 2));
        dPos = sumDPos/(dataPoints-1);

        //difference between drone's position and roomba
        float dx = current_pose.pose.pose.position.x - decks[TARGETQ].front().pose.position.x;
        float dy = current_pose.pose.pose.position.y - decks[TARGETQ].front().pose.position.y;
        float magdydx = sqrt( pow(dx, 2) + pow(dy, 2));

        cout << "dPos " << dPos << endl;
        if(abs(dPos) < dTol)
        {
          waypoint.pose.position.x = decks[TARGETQ].front().pose.position.x + 1.2*(dx/magdydx);
          waypoint.pose.position.y = decks[TARGETQ].front().pose.position.y + 1.2*(dy/magdydx);
          waypoint.pose.position.z = .5;
          MODE = 3;
          cout << "MODE : " << MODE << endl;
          mode3SetTime = ros::Time::now().toSec();

        }

        deltaX = abs(waypoint.pose.position.x - current_pose.pose.pose.position.x);
        deltaY = abs(waypoint.pose.position.y - current_pose.pose.pose.position.y);
        deltaZ = abs(waypoint.pose.position.z - current_pose.pose.pose.position.z);
        //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
        float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );

        if( dMag < tollorance)
        {
          if(magdydx > 2.25)
          {
            ROS_INFO("Moving closer to roomba");
            waypoint.pose.position.x = decks[TARGETQ].front().pose.position.x - 1.5*(dxdt/dPos);
            waypoint.pose.position.y = decks[TARGETQ].front().pose.position.y - 1.5*(dydt/dPos);
            waypoint.pose.position.z = .85;
          }
        }

        //TIME OUT condition
        if (currentTime - mode2SetTime > 20 && MODE == 2 )
        {
          cout << "giving up. Timed out" << endl;
          MODE = 0;
          lastmode0SetTime = ros::Time::now().toSec();
          //clear Q with lost roomba
          ROS_INFO("Resetting Qs");
          for(int i=0; i<decks.size(); i++)
          {
            while (decks[i].size()>1)
            {
              decks[i].pop_back();
            }
          }

          waypoint.pose.position.x = reset_pos_x[counter%3];
          waypoint.pose.position.y = reset_pos_y[counter%3];
          waypoint.pose.position.z = 1.5;
          counter++;
          heading = 0;
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
          ROS_INFO("Resetting Qs");
          for(int i=0; i<decks.size(); i++)
          {
            while (decks[i].size()>1)
            {
              decks[i].pop_back();
            }
          }
          MODE = 0;
          lastmode0SetTime = ros::Time::now().toSec();
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
    // matplotlibcpp::ion();
    // if (roombaPositions.roombaPoses.size())
    // {
    //   matplotlibcpp::xlim(-.5, 19.5);
    //   matplotlibcpp::ylim(-.5, 19.5);


    //   // for (int i=0; i < roombaPositions.roombaPoses.size(); i++){
    //   //    x.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.x);
    //   //    y.push_back(roombaPositions.roombaPoses[i].roombaPose.pose.position.y);
    //   //    matplotlibcpp::plot(x, y, "ro");
    //   //    matplotlibcpp::pause(0.001);
    //   //    matplotlibcpp::draw();
    //   //  }
    //   string colour[10]= {"bo","go","ro","co","mo","yo","ko","bv","b.","rv"};
    //   string special_color= "g+";
    //   for (int j=0; j < decks.size(); j++){
    //       //cout<<decks[j][0]<<endl;
    //       //cout<<colour[j]<<endl;

    //       x[0]=decks[j][0].pose.position.x;
    //       y[0]=decks[j][0].pose.position.y;
    //       if(j==TARGETQ){
    //        matplotlibcpp::plot(x, y, special_color);
    //       }
    //       else{
    //       matplotlibcpp::plot(x, y, colour[j]);
    //     }
    //       matplotlibcpp::pause(0.0001);
    //       matplotlibcpp::draw();

    //   }
    // // if(TARGETQ!=100){
    // //       x[0]=decks[TARGETQ][0].pose.position.x;
    // //       y[0]=decks[TARGETQ][0].pose.position.y;
    // //       matplotlibcpp::plot(x, y, colour[TARGETQ]);
    // //       matplotlibcpp::pause(0.0001);
    // //       matplotlibcpp::draw();
    // //}

    // }

    ++count;
    // if(count%20==0){
    //  matplotlibcpp::clf();
    // }
  }
  return 0;

}
