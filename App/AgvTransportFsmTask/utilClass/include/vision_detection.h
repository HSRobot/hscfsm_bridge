#pragma once
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/PoseStamped.h>

// #include "hirop_msgs/transformFramePose.h"
#include "hirop_msgs/detection.h"
#include "hirop_msgs/setFromIK.h"
#include "hirop_msgs/ObjectArray.h"
#include "hirop_msgs/joints_angle.h"
#include <atomic>

using namespace std;

class VisionDetection
{
public:
    VisionDetection(ros::NodeHandle *nh);
    ~VisionDetection();

    int detection(string object, string detector);
    int compensatin_x(double x);
    int compensatin_y(double y);
    int compensatin_z(double z);
    int getJointAngleResult(vector<double> &joints);

private:
    void objectArrayCB(const hirop_msgs::ObjectArrayConstPtr &msg);
    int transformFrame(const geometry_msgs::PoseStamped &p, geometry_msgs::PoseStamped &target, const string &frame_id);
    int IK(geometry_msgs::PoseStamped &pose, vector<double> &joints, string tip_);
    vector<double> rad2angle(vector<double> rad);

private:
    ros::NodeHandle *nh;
    ros::Publisher anglePub;
    ros::Subscriber objectArraySub;
    ros::ServiceClient detectionClient;

private:
    double wX, wY, wZ;
    string arm, tip, targetFrame;
    vector<double> resultAngle;

    atomic<int> ArrayFlag;
    robot_model::RobotModelPtr kinematic_model;
    // robot_state::RobotStatePtr robot_state;
    robot_state::RobotState *robot_state;
    const robot_model::JointModelGroup *joint_model_group;
};
