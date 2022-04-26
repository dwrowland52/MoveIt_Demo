#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/RobofleetStatus.h"
#include <chrono>
#include "sensor_msgs/NavSatFix.h"

double GetTimeStamp()
{   
    //auto curr_time = std::chrono::system_clock::time_point::now();
    //std::chrono::system_clock::time_point t0 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    //std::chrono::system_clock::duration t0_from_epoch = t0.time_since_epoch();
    //int t0_ms = t0_from_epoch.count();
    //int t0_ms = to_int(curr_time);
    double curr_time = ros::WallTime::now().toSec();

    return curr_time;
}

void RobofleetStatusCallback(ros::Publisher status_pub)
{
    amrl_msgs::RobofleetStatus status;
    status.battery_level=0.9;
    status.is_ok = true;
    status.status = "Grounded";
    status.location = "O_NRG";
    status_pub.publish(status);
}

void RobofleetLocationCallback(ros::Publisher location_pub, float x, float y)
{
    amrl_msgs::Localization2DMsg location;
    double curr_time = GetTimeStamp();
    location.header.frame_id = "Grounded";
    location.pose.x = x;
    location.pose.y = y;
    location.pose.theta = 10;
    location_pub.publish(location);
}

void RobofleetNavSatFixCallback(ros::Publisher navsatfix_pub)
{
    sensor_msgs::NavSatFix navsatfix;
    navsatfix.header.frame_id = "spoof";
    /* Status Description
    int8 STATUS_NO_FIX=-1
    int8 STATUS_FIX=0
    int8 STATUS_SBAS_FIX=1
    int8 STATUS_GBAS_FIX=2
    */
    navsatfix.status.status = 0;
    /* Service Description
    uint16 SERVICE_GPS=1
    uint16 SERVICE_GLONASS=2
    uint16 SERVICE_COMPASS=4
    uint16 SERVICE_GALILEO=8
    */
    navsatfix.status.service = 1;
    // NRG Lab Approximate Coordinates from Google Maps: Lat: 30.28816215895828, Lon: -97.73756667374789
    navsatfix.latitude = 30.28816215895828;
    navsatfix.longitude = -97.73756667374789;
    navsatfix.altitude = 10;
    std::array<double,9> position_covariance {1,0,0,0,1,0,0,0,1};
    std::copy(position_covariance.begin(),position_covariance.end(),navsatfix.position_covariance.begin());
    /* Covariance Type Description
    uint8 COVARIANCE_TYPE_UNKNOWN=0
    uint8 COVARIANCE_TYPE_APPROXIMATED=1
    uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
    uint8 COVARIANCE_TYPE_KNOWN=3
    */
    navsatfix.position_covariance_type = 1;
    navsatfix_pub.publish(navsatfix);
}


int main(int argc, char **argv)
{
    // Intialize Topic Name and Node Handle
    ros::init(argc,argv,"spoofed_robot_pub");
    ros::NodeHandle n;
    
    // Setup Ros Publishers
    ros::Publisher status_pub = n.advertise<amrl_msgs::RobofleetStatus>("/status", 1);
    ros::Publisher location_pub = n.advertise<amrl_msgs::Localization2DMsg>("/localization", 1);
    //ros::Publisher hololens_status_pub = n.advertise<amrl_msgs::RobofleetStatus>("/U_NRG/status", 1);
    //ros::Publisher hololens_location_pub = n.advertise<amrl_msgs::Localization2DMsg>("/U_NRG/localization", 1);
    //ros::Publisher navsatfix_pub = n.advertise<sensor_msgs::NavSatFix>("/NavSatFix", 1);

    // Set Publishing Rate
    ros::Rate loop_rate(20);
    float iter = 0;

    // Publish Messages while Node is Running
    while (ros::ok()) {
        

        // Publish Status Message
        RobofleetStatusCallback(status_pub);
        //RobofleetStatusCallback(hololens_status_pub);

        // Publish Location Message
        RobofleetLocationCallback(location_pub, 0, 0);
        //RobofleetLocationCallback(hololens_location_pub, 2, -1);
        // Publish NavSatFix Message
        // RobofleetNavSatFixCallback(navsatfix_pub);

        double msg_time_stamp = GetTimeStamp();
        ROS_INFO("Robot Published [MsgTimeStamp: %f]", msg_time_stamp);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}