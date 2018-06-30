#include "ros/ros.h"
#include "std_msgs/String.h"
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

using namespace std;

int MAX_POINTS = 10;
int marker=0; 

geometry_msgs::PoseStamped waypoint;
transformations_ros::roombaPoses roombaPositions;
vector<deque<geometry_msgs::PoseStamped>> decks;
double dist=90000;
double temp;
struct slope
{
  float mxdt;
  float mydt;
};
void roomba_cb(const transformations_ros::roombaPoses::ConstPtr& msg)
{
 roombaPositions = *msg;


     if (decks.size()== 0){
      cout<<"URMOM"<<endl;
       decks.push_back(deque<geometry_msgs::PoseStamped>());
       decks[0].push_front(roombaPositions.roombaPoses[0].roombaPose);
    }

 for (int i=0; i < roombaPositions.roombaPoses.size(); i++){
  dist=9000;
  int rpt = 0;
  int ctr=0;

    for (int j=0; j < decks.size(); j++){

      temp =sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2));
      if(temp<dist){
        dist=temp;
        ctr=j;
      }
       
    }
    if ( dist<= .8){
          //Found roomba close to deck[j] and adding it to the queue
          decks[ctr].push_front(roombaPositions.roombaPoses[i].roombaPose);
          cout << "roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << ctr<< endl;
            cout<< "Located "<<dist << "away from nearest queue"<<endl;
          if(decks[ctr].size() >= MAX_POINTS){decks[ctr].pop_back();}
        }else{
           cout<< "Located "<<dist << "away from nearest queue"<<endl;
           if(decks.size()<10){ 
           decks.push_back(deque<geometry_msgs::PoseStamped>());
           decks[decks.size()-1].push_front(roombaPositions.roombaPoses[i].roombaPose);
           cout << "New roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << decks.size()-1 << endl;
         }else{
     
          while (decks[marker].size()>1)
              { 
                
                decks[marker].pop_back();
              }
           
           decks[marker].push_front(roombaPositions.roombaPoses[i].roombaPose);

           cout<< "No queue matches current roomba at FUCKU"<<endl;

         }

          // case 1:
          //  int min = 0;
          //  cout<<decks.size();
          //  vector<double> score(decks.size());
          //  for (int k = 0; k < score.size(); k++){
          //    score[k] = sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[k].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[k].front().pose.position.y,2));
          //      if (k > 0){
          //       if (score[k] < score[k-1]){
          //         min = k;
          //       }
          //     }  
          //   }

          //    if (decks[min].size() <= MAX_POINTS){
          //      decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
          //      cout << "detection at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << " belongs to roomba #" << min << endl;
          //    }
          //    else {
          //      decks[min].pop_back();
          //      decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
          //      cout << "detection at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << " belongs to roomba #" << min << endl;
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
        // cout <<"nt: " << nt << endl;
        // cout <<"sx: " << sx << endl;
        // cout <<"sy: " << sy << endl;
        // cout <<"st: " << st << endl;
        // cout <<"sxt: " << sxt << endl;
        // cout <<"syt: " << syt << endl;
        // cout <<"sx2: " << sx2 << endl;
        // cout <<"sy2: " << sy2 << endl;
        // cout <<"nt: " << nt << endl;
        //cout<<endl<<"point "<< points[i]<<endl;
     }

     cout<<"delta T in use: "<<st<<endl;
     cout<<"the back x point is "<<points.back().pose.position.x<<endl;
     
    //ix = (sx*st2 - st*sxt)/((points.size()-1)*st2-(st*st));
    //iy = (sy*st2 - sy*syt)/((points.size()-1)*st2-(st*st));
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
      cout<<marker<<endl;
     }


    cout << "DT "; 
    for (int j=decks[i].size()-1; j>=0; j--)
    {
      dt2 = currentTime_d - decks[i][j].header.stamp.toSec();
      cout << " " << dt2;
    }
    cout << " " << endl;


  }
    


  

  cout << "dT " << dt << endl;
}

void target()
{
  for(int i=0; i<decks.size(); i++)
  {
    if (decks[i].size() > 4)
    {
      slope currentSlope;
      currentSlope = linreg(decks[i]);
      cout << "current slope x " << currentSlope.mxdt << " y " << currentSlope.mydt <<endl;
      float dydx = currentSlope.mydt * (1/currentSlope.mxdt);
      float xInt = dydx*(decks[i].front().pose.position.y - (-.5)) +  decks[i].front().pose.position.x;
      if (xInt > -.5 && xInt < 19.5 && currentSlope.mydt < 0)
      {
        cout << "Possible target roomba#" << i << " mydt " << currentSlope.mydt << endl;
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
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1);

  ros::Rate loop_rate(100);

  waypoint.pose.position.x = 0;
  waypoint.pose.position.y = 7.5;
  waypoint.pose.position.z = .5;
  int mode = 0;
  int count = 0;
  while (ros::ok())
  {

    //check if data has been recieved
    if (decks.size() > 0)
    {
      //Clear old decks
      timeCheck();

      //decide if there is enough data to execute an interaction 
      for(int i=0; i<decks.size(); i++)
      {
        if (decks[i].size() > 4)
        {
          target();
        }
      }
    }
    ros::spinOnce();

    loop_rate.sleep();
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
     
      for (int j=0; j < decks.size(); j++){
          //cout<<decks[j][0]<<endl;
          //cout<<colour[j]<<endl;
        
          x[0]=decks[j][0].pose.position.x;
          y[0]=decks[j][0].pose.position.y;
          matplotlibcpp::plot(x, y, colour[j]);
          matplotlibcpp::pause(0.0001);
          matplotlibcpp::draw(); 

      }

    }

    chatter_pub.publish(waypoint);
    ++count;
    if(count%20==0){
     matplotlibcpp::clf();
    }
  }
  return 0;

}
