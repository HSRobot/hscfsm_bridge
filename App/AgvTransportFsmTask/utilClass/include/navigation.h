#pragma once
#include <ros/ros.h>
#include <hirop/datamanager/file_datawriter.h>
#include <hirop/datamanager/posedata.h>
#include <hirop/nav/mobile_robot.h>
#include <hirop/nav/autorunmotion.h>
#include <actionlib_msgs/GoalID.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>

// #include "timer.h"

using namespace std;
using namespace hirop::data_manager;
using namespace hirop::navigation;

class Navigation
{
public:
    Navigation(ros::NodeHandle* nh);
    // ~Navigation();

    int moveTo(string flag);
    int clearCostMap();
    int mappingIteration(int iter);
    int stop();

private:
    ros::Publisher cancelPub;
    ros::ServiceClient request_nomotion_update_client;

private:
    PoseData *getMapFlagPose(std::string flag);
    
private:
    ros::NodeHandle* nh;
    bool stopFlag;
    string currentMap;
};
