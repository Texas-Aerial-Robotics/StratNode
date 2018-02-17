#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include <string.h>
#include <sstream>
#include <vector>

std::string rootNode = "/stratnode";
std::string objTopic = rootNode + "/objects";


gazebo_msgs::ModelStates curModelStates;

std::vector<ros::Publisher> objPubs; 
std::vector<std::string> objectNames;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

    curModelStates = (gazebo_msgs::ModelStates)(*msg);
    int count =  curModelStates.name.size();

    // for(int i=0; i<count; i++){
    //     ROS_INFO("New Model State: %s\nPosition: %f, %f, %f\nTwist: %f, %f, %f; %f, %f, %f\n\n",
    //          curModelStates.name[i].c_str(),
    //          curModelStates.pose[i].position.x,
    //          curModelStates.pose[i].position.y,
    //          curModelStates.pose[i].position.z,
    //          curModelStates.twist[i].linear.x,
    //          curModelStates.twist[i].linear.y,
    //          curModelStates.twist[i].linear.z,
    //          curModelStates.twist[i].angular.x,
    //          curModelStates.twist[i].angular.y,
    //          curModelStates.twist[i].angular.z);
    // }



}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "stratnode");

    ros::NodeHandle nhStrat;
    // ros::NodeHandle nhObj(nh_strat, "objects");


    ros::Subscriber state_sub = nhStrat.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, modelStatesCb);


    ros::Duration(5.0).sleep();

    ros::spinOnce();

    int count =  curModelStates.name.size();
    ROS_INFO("Current model states: %d\n", count);

    for(int i=0; i<count; i++){

        std::string name = curModelStates.name[i].c_str();
        std::string ptopic = rootNode + "/" + curModelStates.name[i].c_str() + "/pose";
        std::string ttopic = rootNode + "/" + curModelStates.name[i].c_str() + "/twist";
        ros::Publisher state_pose_pub = nhStrat.advertise<geometry_msgs::Pose>(ptopic, 10, true);
        ros::Publisher state_twist_pub = nhStrat.advertise<geometry_msgs::Twist>(ttopic, 10, true);
        objPubs.push_back(state_pose_pub);
        objPubs.push_back(state_twist_pub);
        objectNames.push_back(name);
        objectNames.push_back(name);
        ROS_INFO ("Adding Publisher %s", name);

    }

    ROS_INFO ("Added %d publishers", objPubs.size());
 

    ros::Rate loop_rate(10.0);

    // int count = 0;
    while (ros::ok()){

        ros::spinOnce();

        count =  curModelStates.name.size();
        ROS_INFO("Current model states: %d\n", count);
        for(int i=0; i<count; i++){
            ROS_INFO("New Model State: %s\nPosition: %f, %f, %f\nTwist: %f, %f, %f; %f, %f, %f\n\n",
                 curModelStates.name[i].c_str(),
                 curModelStates.pose[i].position.x,
                 curModelStates.pose[i].position.y,
                 curModelStates.pose[i].position.z,
                 curModelStates.twist[i].linear.x,
                 curModelStates.twist[i].linear.y,
                 curModelStates.twist[i].linear.z,
                 curModelStates.twist[i].angular.x,
                 curModelStates.twist[i].angular.y,
                 curModelStates.twist[i].angular.z);

            geometry_msgs::Pose tempPose = curModelStates.pose[i];
            geometry_msgs::Twist tempTwist = curModelStates.twist[i];
            ros::Publisher state_pose_pub = objPubs[i*2];
            ros::Publisher state_twist_pub = objPubs[i*2+1];
            state_pose_pub.publish(tempPose);
            state_twist_pub.publish(tempTwist);
        }


        loop_rate.sleep();
        ++count;
    }

    return 0;
}

