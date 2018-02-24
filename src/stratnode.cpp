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


    // ros::Duration(5.0).sleep();

    // ros::spinOnce(); 

    ros::Rate loop_rate(10.0);

    // int count = 0;
    while (ros::ok()){

        ros::spinOnce();

        int modelCount = curModelStates.name.size();
        int pubCount = objPubs.size();


        ROS_INFO("Current model states: %d\n", modelCount);
        for(int i=0; i<modelCount; i++){

            char* name = curModelStates.name[i].c_str();
            geometry_msgs::Pose tempPose = curModelStates.pose[i];
            geometry_msgs::Twist tempTwist = curModelStates.twist[i];

            int pubIndex = -1;
            for(int j=0; j<pubCount/2; j++){
                if(objectNames[i] == name){
                    pubIndex = j*2;
                }
            }

            if(pubIndex == -1){
                std::string ptopic = rootNode + "/" + curModelStates.name[i].c_str() + "/pose";
                std::string ttopic = rootNode + "/" + curModelStates.name[i].c_str() + "/twist";
                ros::Publisher state_pose_pub = nhStrat.advertise<geometry_msgs::Pose>(ptopic, 10, true);
                ros::Publisher state_twist_pub = nhStrat.advertise<geometry_msgs::Twist>(ttopic, 10, true);
                objPubs.push_back(state_pose_pub);
                objPubs.push_back(state_twist_pub);
                objectNames.push_back(name);
                pubIndex = objPubs.size() - 1;
                pubCount += 2;
                ROS_INFO ("Adding Publisher %s", name);
            }


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

            ros::Publisher state_pose_pub = objPubs[pubIndex];
            ros::Publisher state_twist_pub = objPubs[pubIndex+1];
            state_pose_pub.publish(tempPose);
            state_twist_pub.publish(tempTwist);
        }


        loop_rate.sleep();
        // ++count;
    }

    return 0;
}

