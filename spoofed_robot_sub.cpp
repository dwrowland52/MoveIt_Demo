#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/RobofleetStatus.h"
#include <chrono>
#include "sensor_msgs/NavSatFix.h"


void ReturnMessageCallback(const amrl_msgs::RobofleetStatus msg)
{
    double curr_time = ros::WallTime::now().toSec();

    std::string incoming_start_time = msg.status;
    double start_time = ::atof(incoming_start_time.c_str());
    ROS_INFO("Start Time Stamp: %s; End Time Stamp: %s", incoming_start_time.c_str(), std::to_string(curr_time).c_str());
    
    double dur = curr_time - start_time;
    ROS_INFO("Duration: %s", std::to_string(dur).c_str());

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"spoofed_robot_sub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/U_Regal/status",1000,ReturnMessageCallback);
    ros::spin();
    return 0;
}