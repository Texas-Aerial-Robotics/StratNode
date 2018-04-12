#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>

#include <string>
#include <fstream>
#include <time.h>
#include <ctime>

gazebo_msgs::ModelStates curModelStates;

void init_datafile(std::String file_name)
{
  std::ofstream outTimeHist(file_name.c_str(),std::ios::app);

  outTimeHist << "Time(ms), Object Name, position_X(m), position_Y(m), position_Z(m/s), velocity_X(m/s), velociy_Y(m/s), velocity_Z(m/s)"

  outTimeHist.close();
}

void model_states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  curModelStates = *msg;
}

const std::string currentDateTime()
{
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf,sizeof(buf),"%m_%d_%y-%H_%M", &tstruct);
  return buf;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Gazebo_model_data");
  ros::NodeHandle gaz_data;

  ros::Subscriber imu_temp = gaz_data.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states",10,model_states_cb);

  std::string date_file = "/sdCard/Logs/"+currentDateTime()+"_Gazebo_data.csv";

  init_datafile(date_file);

  struct timeval start,stop;
  gettimeofday(&start,NULL);
  long int ms_start = start.tv_sec * 1000 + start.tv_usec / 1000;
  while (ros::ok())
  {
    ros::spinOnce();
    std::ofstream outTimeHist(date_file.c_str(),std::ios::app);

    int count = curModelStates.name.size();
   
    gettimeofday(&stop,NULL); 
    long int ms_stop = stop.tv_sec * 1000 + stop.tv_usec / 1000;
    
    for(int i=0;i<count; i++)
    {
      outTimeHist << ms_stop-ms_start << "," << curModelStates.name[i].c_str() << ",";
      outTimeHist << curModelStates.pose[i].position.x << "," << curModelStates.pose[i].position.y << "," << curModelStates.pose[i].position.z;
      outTimeHIst << curModelStates.twist[i].linear.x << "," << curModelStates.twist[i].linear.y << "," << curModelStates.twist[i].linear.z;
    }
    outTimeHist.close();
  }
}
