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

geometry_msgs::PoseStamped waypoint;
transformations_ros::roombaPoses roombaPositions;
vector<deque<geometry_msgs::PoseStamped>> decks;

void roomba_cb(const transformations_ros::roombaPoses::ConstPtr& msg)
{
 roombaPositions = *msg;

 cout<<roombaPositions;

 for (int i=0; i < roombaPositions.roombaPoses.size(); i++){

  int rpt = 0;
    if (decks.size()== 0){
      cout<<"URMOM"<<endl;
       decks.push_back(deque<geometry_msgs::PoseStamped>());
       decks[0].push_front(roombaPositions.roombaPoses[i].roombaPose);
    }
    for (int j=0; j < decks.size(); j++){
       if (sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2)) <= .5){
        rpt = 1;
      }
    }
        switch(rpt){ 

          case 0:
           decks.push_back(deque<geometry_msgs::PoseStamped>());
           decks[decks.size()-1].push_front(roombaPositions.roombaPoses[i].roombaPose);
           cout << "New roomba detected at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << ". This is roomba #" << decks.size()-1 << endl;

          case 1:
           int min = 0;
           cout<<decks.size();
           vector<double> score(decks.size());
           for (int k = 0; k < score.size(); k++){
             score[k] = sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[k].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[k].front().pose.position.y,2));
               if (k > 0){
                if (score[k] < score[k-1]){
                  min = k;
                }
              }  
            }

             if (decks[min].size() <= MAX_POINTS){
               decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
               cout << "detection at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << " belongs to roomba #" << min << endl;
             }
             else {
               decks[min].pop_back();
               decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
               cout << "detection at " << roombaPositions.roombaPoses[i].roombaPose.pose.position.x << ", " << roombaPositions.roombaPoses[i].roombaPose.pose.position.y << " belongs to roomba #" << min << endl;
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

  int count = 0;
  while (ros::ok())
  {
    
    //cout << points << endl;
    ros::spinOnce();
    loop_rate.sleep();
    matplotlibcpp::ion();
    if (roombaPositions.roombaPoses.size())
    {
      matplotlibcpp::xlim(-10, 10);
      matplotlibcpp::ylim(-10, 10);
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
          matplotlibcpp::pause(0.001);
          matplotlibcpp::draw(); 

      }
    }
    chatter_pub.publish(waypoint);
    ++count;
  }
  return 0;
}
