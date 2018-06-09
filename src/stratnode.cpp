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

using namespace std;

int MAX_POINTS = 10;
std::vector<double> x, y;
geometry_msgs::PoseStamped waypoint;
transformations_ros::roombaPoses roombaPositions;
//deque<geometry_msgs::PoseStamped> points;
vector<deque<geometry_msgs::PoseStamped>> decks;

void roomba_cb(const transformations_ros::roombaPoses::ConstPtr& msg)
{
 roombaPositions = *msg;

 for (int p = 0; p <= roombaPositions.roombaPoses.size(); p++){
    roombaPositions.roombaPoses[p].roombaPose.pose.position.z=1.5;
    for (int m = 0; m <= decks.size(); m ++){
       if (roombaPositions.roombaPoses[p].roombaPose.pose.position.x == decks[m].front().pose.position.x && roombaPositions.roombaPoses[p].roombaPose.pose.position.y == decks[p].front().pose.position.y){
        break;
      }
    }
  }
 for (int i=0; i < roombaPositions.roombaPoses.size(); i++){
    for (int j=0; j < decks.size(); j++){
       if (sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2)) >= 3){
         decks.push_back(deque<geometry_msgs::PoseStamped>());
         decks[decks.size()-1].push_front(roombaPositions.roombaPoses[i].roombaPose);
       }
       else{
        for (int i=0; i < roombaPositions.roombaPoses.size(); i++)
        {
          int min = 0;
         vector<double> score(decks.size());
           for (int j = 0; j < score.size(); j ++){
             score[j] = sqrt(pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.x - decks[j].front().pose.position.x,2) + pow(roombaPositions.roombaPoses[i].roombaPose.pose.position.y - decks[j].front().pose.position.y,2));
               if (j > 0){
                if (score[j] < score[j-1]){
                  min = j;
                }
              }  
           }
          if (decks[min].size() <= MAX_POINTS){
            decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
          }
          else {
            decks[min].pop_back();
            decks[min].push_front(roombaPositions.roombaPoses[i].roombaPose);
          }
        }
      }
    }
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stratnode");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<transformations_ros::roombaPoses>("roombaPoses", 1, roomba_cb);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1);

  ros::Rate loop_rate(10);

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
      x.push_back(roombaPositions.roombaPoses[0].roombaPose.pose.position.x);
      y.push_back(roombaPositions.roombaPoses[0].roombaPose.pose.position.y);
      matplotlibcpp::plot(x, y, "ro");
      matplotlibcpp::pause(0.001);
      matplotlibcpp::draw();
      //chatter_pub.publish(waypoint);
      //cout<<points.size()<< endl;
      //for (int i=0;i++;i<points.size())
      {
        //cout << points[i] << endl;
      }
    }
    ++count;
  }
  return 0;
}
