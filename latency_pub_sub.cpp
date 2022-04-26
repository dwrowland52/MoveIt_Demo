#include <iostream>
#include <sstream>
#include <chrono>
#include <fstream>

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/RobofleetStatus.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

class AugRELatencyStudy
{
private:
    std::map<std::string, ros::Publisher> RosPublisherMap;
    std::vector<std::map<std::string, ros::Publisher>> RosPublisher;
    std::chrono::_V2::steady_clock::time_point PulseStopTime;
    std::chrono::_V2::steady_clock::time_point RoundTripStartTime;
    std::chrono::_V2::steady_clock::time_point RoundTripStopTime;
    double PulseDurationTime;
    double RoundTripDurationTime;
    double GetTimeDuration(std::chrono::_V2::steady_clock::time_point StartTime, std::chrono::_V2::steady_clock::time_point StopTime);
    float CheckDistance;
    float SumDistance;
    float DistX;
    bool RoundTripComplete;
    std::vector<float> DistY;
    int LogTestNumber;
    bool IsFirstTripMessageSent;

public:
    AugRELatencyStudy();

    int NumberOfTestRobots;             // Number of Clients to Publish - Remember to Update You Local Robofleet Client Config File
    std::string RobotBaseName;          // Must have "2D_" prefix
    float RobotClientPublishingRate;    // [Hz]
    float HololensClientPublishingRate; // [Hz] grab from BP_TestRoom1 EventGraph in AugRE App
    std::string OutputFileName;
    std::string OutputFileLocalPath;
    int PulseTimerInterval;
    int NumberOfTests;                  // set the number of tests you would like to do
    bool KeepRunning;

    std::chrono::_V2::steady_clock::time_point LogStartTime();
    std::chrono::_V2::steady_clock::time_point LogStopTime();
    std::chrono::_V2::steady_clock::time_point PulseStartTime;
    void InitPublisher(ros::NodeHandle n);
    void RobofleetStatusCallback(ros::Publisher status_pub);
    void RobofleetLocationCallback(ros::Publisher location_pub, float PoseY);
    void PublishTestMessage();
    void ReturnStatusMessageCallback(const amrl_msgs::RobofleetStatus msg);
    void ReturnLocationMessageCallback(const amrl_msgs::Localization2DMsg msg);
    void SetOutputFileDirectory(std::string LocalPath);
    void SetupOutputFile();
    void BuildYvector();
    void InitPublisherMessages();
};

AugRELatencyStudy::AugRELatencyStudy()
    : NumberOfTestRobots{1}, 
      DistX{10},
      RobotBaseName{"2D_robot"}, 
      CheckDistance{NumberOfTestRobots*DistX},
      SumDistance{0},
      RoundTripComplete{true},
      OutputFileLocalPath{"~/home/"},
      RobotClientPublishingRate{10},
      HololensClientPublishingRate{10},
      OutputFileName{"LatencyStudyResults.txt"},
      PulseTimerInterval{15},
      LogTestNumber{0},
      NumberOfTests{10},
      KeepRunning{true}
{ // Default Constructor
}

void AugRELatencyStudy::BuildYvector()
{
    DistY.clear();
    float posdist = 0;
    float negdist = 0;
    float spread = 0.5; // [m]
    for (int i = 0; i < NumberOfTestRobots; i++)
    {
        if (i % 2 == 0)
        {
            DistY.push_back(posdist);
            posdist = posdist + spread;
        } else 
        {
            negdist = negdist + spread;
            DistY.push_back(-negdist);
        }
    }
}

std::chrono::_V2::steady_clock::time_point AugRELatencyStudy::LogStartTime()
{
    std::chrono::_V2::steady_clock::time_point StartTime = std::chrono::steady_clock::now();
    return StartTime;
}

std::chrono::_V2::steady_clock::time_point AugRELatencyStudy::LogStopTime()
{
    std::chrono::_V2::steady_clock::time_point StopTime = std::chrono::steady_clock::now();
    return StopTime;
}

double AugRELatencyStudy::GetTimeDuration(std::chrono::_V2::steady_clock::time_point StartTime, std::chrono::_V2::steady_clock::time_point StopTime)
{
    std::chrono::duration<double> diff = StopTime - StartTime;
    double TimeDuration = diff.count();
    return TimeDuration;
}

void AugRELatencyStudy::InitPublisher(ros::NodeHandle n)
{
    RosPublisher.clear();
    CheckDistance = NumberOfTestRobots * DistX;
    std::cout << "Check Distance: " << CheckDistance << std::endl;
    for (int i = 0; i < NumberOfTestRobots; i++)
    {
        RosPublisherMap["status"] = n.advertise<amrl_msgs::RobofleetStatus>("/" + RobotBaseName + std::to_string(i+1) + "/status", 1);
        RosPublisherMap["localization"] = n.advertise<amrl_msgs::Localization2DMsg>("/"+ RobotBaseName + std::to_string(i+1) + "/localization", 1);
        RosPublisher.push_back(RosPublisherMap);
    }
    
}

void AugRELatencyStudy::RobofleetStatusCallback(ros::Publisher status_pub)
{
    amrl_msgs::RobofleetStatus status;
    status.battery_level=70;
    status.is_ok = true;
    status.status = "Good";
    status.location = "O_Test";
    status_pub.publish(status);
}

void AugRELatencyStudy::RobofleetLocationCallback(ros::Publisher location_pub, float PoseY)
{
    amrl_msgs::Localization2DMsg location;
    //location.header.frame_id = "NaN";
    location.pose.x = DistX;
    location.pose.y = PoseY;
    location.pose.theta = 0;
    location_pub.publish(location);
}

void AugRELatencyStudy::ReturnStatusMessageCallback(const amrl_msgs::RobofleetStatus msg)
{
    LogStopTime();
    ROS_INFO("Status Message Received");
    
}

void AugRELatencyStudy::ReturnLocationMessageCallback(const amrl_msgs::Localization2DMsg msg)
{
    if (abs(msg.pose.x-CheckDistance) <= 5 && !RoundTripComplete) // May not be best implementation as Robofleet Drops messages
    {   
        RoundTripStopTime = LogStopTime();
        //SumDistance = 0;
        RoundTripComplete = true;
        PulseStartTime = LogStartTime();
        std::string HoloLensFPS = msg.header.frame_id;
        HololensClientPublishingRate = msg.pose.theta;
        RoundTripDurationTime = GetTimeDuration(RoundTripStartTime, RoundTripStopTime);

        std::string OutputLine =  std::to_string(RoundTripDurationTime) + ", " +
                                  std::to_string(NumberOfTestRobots) + ", " + 
                                  std::to_string(RobotClientPublishingRate) + ", " +
                                  std::to_string(HololensClientPublishingRate) + ", " +
                                  HoloLensFPS;
        
        // Write to File
        std::ofstream out_file {OutputFileLocalPath + OutputFileName, std::ios::app};
        if (!out_file) {
            std::cerr << "Error creating file" << std::endl;
            return;
        }
        out_file << OutputLine << std::endl;
        out_file.close();

        std::cout << "\n*** ROUND TRIP COMPLETE ***\n" << std::endl;

        LogTestNumber++;
        if (LogTestNumber >= NumberOfTests)
        {
            KeepRunning = false;
        }
       
        
    }

}

void AugRELatencyStudy::InitPublisherMessages()
{
        ros::Rate loop_rate(0.5); // [Hz]
        int iter = 0;
        DistX = 0;
        for (auto PublishMsg : RosPublisher)
        {   
            //std::cout << "publishing messages: X = 0" << std::endl;
            RobofleetStatusCallback(PublishMsg["status"]);
            RobofleetLocationCallback(PublishMsg["localization"], DistY.at(iter));
            iter++;
            loop_rate.sleep();
        }
}

void AugRELatencyStudy::PublishTestMessage()
{
    // Pulse Messages Every So Many Seconds
    PulseStopTime = LogStopTime();
    PulseDurationTime = GetTimeDuration(PulseStartTime,PulseStopTime);
    int iter = 0;
    if (PulseDurationTime < PulseTimerInterval && RoundTripComplete) // Must be greater than the average response time
    {
        //std::cout << "Pulse Timer: " << PulseDurationTime << std::endl; // debug
        DistX = 0;
        //SumDistance = 0;
        for (auto PublishMsg : RosPublisher)
        {   
            //std::cout << "publishing messages: X = 0" << std::endl;
            RobofleetStatusCallback(PublishMsg["status"]);
            RobofleetLocationCallback(PublishMsg["localization"], DistY.at(iter));
            iter++;
        }
        iter = 0;
        IsFirstTripMessageSent = false;
    } 
    else 
    {
        DistX = 10;
        RoundTripComplete = false;
        if (!IsFirstTripMessageSent)
        {
            RoundTripStartTime = LogStartTime();
            IsFirstTripMessageSent = true;
        }
        for (auto PublishMsg:RosPublisher)
        {   
            RobofleetStatusCallback(PublishMsg["status"]);
            RobofleetLocationCallback(PublishMsg["localization"],DistY.at(iter));
            //std::cout <<"Pulse Timer: " << PulseDurationTime << " [PUBLISHED MSG # " << iter+1 << "; X = 10]" <<std::endl; // debug
            iter++;
        }
        iter = 0;
        //std::cout << std::endl; // debug
    }

}

void AugRELatencyStudy::SetupOutputFile()
{
    // Write to File
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y.%m.%d_%H%M%S_");

    OutputFileName = ss.str() + OutputFileName;
    std::ofstream out_file {OutputFileLocalPath + OutputFileName, std::ios::app};

    if (!out_file) {
        std::cerr << "Error creating file" << std::endl;
        return;
    }
    std::string OutputLine = "Round Trip Time Duration [s], Number of Robot Clients, Robot Client Publishing Rate [Hz], Hololens Return Publishing Rate [Hz], Hololens FPS";
    out_file << OutputLine << std::endl;
    out_file.close();
}

int main (int argc, char **argv)
{
    // Intialize Topic Name and Node Handle
    ros::init(argc,argv,"latency_pub_sub");
    ros::NodeHandle n;
    
    
    // Create a class object
    AugRELatencyStudy LatencyTest;

    // =========================================================================================================================
    // Edit Params In-Between LineBreaks

    // Set Study Parameters (Will be Printed in Output File)
    //n.param("NumOfRobots", LatencyTest.NumberOfTestRobots, 1);
    
    
    LatencyTest.NumberOfTestRobots = 10;            // how many robots you want; *** Remember to update your Robofleet Client Config file ***
    LatencyTest.RobotBaseName = "2D_robot";        // Must have "2D_" prefix; *** Remember to update your Robofleet Client Config file ***
    LatencyTest.RobotClientPublishingRate = 10;    // [Hz]
    //LatencyTest.HololensClientPublishingRate = 20; // [Hz] grab from BP_TestRoom1 EventGraph in AugRE App
    LatencyTest.NumberOfTests = 101;                // Set the Number of Tests you want to run

    // Set Output File Name and Path
    LatencyTest.OutputFileLocalPath = "/mnt/c/Users/regal/devel/github_nrg_private/ROS_pkgs/src/augre_demo_support/src/results_test5/10_hz_tests/";
    LatencyTest.OutputFileName = "LatencyStudy.txt";

    // Call after all your wanted parameters are set
    LatencyTest.SetupOutputFile();

    // Indicate how often you want moved messages published
    LatencyTest.PulseTimerInterval = 2; // [s]

    // Set your subscriber topic "i.e. /hololens/localization"; If Changed must also change in AugRE BP_TestRoom1 EventGraph
    ros::Subscriber LocationSub = n.subscribe("/hololens/localization", 1000, &AugRELatencyStudy::ReturnLocationMessageCallback, &LatencyTest);

    // Create some spacing in the y
    LatencyTest.BuildYvector(); // make the menus spread apart in the app
    // =========================================================================================================================

    // Setup Publishers
    LatencyTest.InitPublisher(n);
    //LatencyTest.InitPublisherMessages();

    // Start your Publishing
    ros::Rate loop_rate(LatencyTest.RobotClientPublishingRate); // Publishing Rate
   
    LatencyTest.PulseStartTime = LatencyTest.LogStartTime();


    while (ros::ok() && LatencyTest.KeepRunning) 
    {
        LatencyTest.PublishTestMessage();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}