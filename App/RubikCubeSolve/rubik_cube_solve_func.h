#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Bool.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

#include "rubik_cube_solve/rubik_cube_solve_cmd.h"
#include "rubik_cube_solve/recordPoseStamped.h"
#include "cubeParse/TakePhoto.h"
#include "cubeParse/SolveCube.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"
#include "rb_msgAndSrv/rb_ArrayAndBool.h"

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

/** 夹爪 **/
#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
/****/

/** motion_bridge **/
#include "hirop_msgs/moveToSiglePose.h"
#include "hirop_msgs/moveToMultiPose.h"
#include "hirop_msgs/moveLine.h"
#include "hirop_msgs/moveSigleAixs.h"
#include "hirop_msgs/motionBridgeStart.h"
/****/

/** dm_bridge **/
#include "hirop_msgs/addJointPose.h"
#include "hirop_msgs/addPose.h"
#include "hirop_msgs/savePoseData.h"
#include "hirop_msgs/saveDataEnd.h"
#include "hirop_msgs/loadJointsData.h"
#include "hirop_msgs/loadPoseData.h"
/****/

/** planner_bridge**/
// #include "hirop_msgs/addPose.h"
#include "hirop_msgs/addJointPose.h"
#include "hirop_msgs/getTrajectory.h"
#include "hirop_msgs/updateParam.h"
/****/


#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "yaml-cpp/yaml.h"

// 記錄三個空間點的以末端坐標系的数据变化
struct RecordPoseData
{
    double pose0[6];
    double pose1[6];
    double pose2[6];
};

struct CubeState
{
    // 0号机或1号机
    int captureRobot;
    // 1,2,3,4个点
    int capturePoint;
    // 在一号空间能拧动的位置
    int canRotateFace1;
    // 在二号空间能拧动的位置
    int canRotateFace2;
    bool isFinish = false;
};

struct ActionData
{
    int captureRobot;
    int otherRobot;
    int capturePoint;
    int space;
    double angle;
    bool isSwop;
};

struct recoredPoint
{
    int model;
    int stepNum;
    int stepCount;
    const std::vector<std::string> poseName={"Pose0", "pose14", "pose03", "pose123", "pose021", "pose022",\
                                    "pose024", "pose025", "pose111", "pose012", "pose023", "pose121", \
                                    "pose122", "pose124", "pose125", "pose011", "pose112", "Pose9"};

    const std::vector<std::string> shootPhotoPoseName = {"Pose1", "Pose2", "Pose3", "Pose4", "Pose5", "Pose6",  "Pose7", "Pose8"};
};

class RubikCubeSolve
{
public:
    RubikCubeSolve(ros::NodeHandle n);
    ~RubikCubeSolve();
    // 多次規劃和move, 減少失敗的概率
    moveit::planning_interface::MoveItErrorCode moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                moveit::planning_interface::MoveGroupInterface::Plan& my_plan);

    // 角度轉爲弧度
    double angle2rad(double& angle);
    // 設置joint_6的角度
    void setJoint6Value(moveit::planning_interface::MoveGroupInterface& move_group, int angle,bool joint_mode =true);
    // 循環執行,防止控制器不響應
    moveit::planning_interface::MoveItErrorCode loop_move(moveit::planning_interface::MoveGroupInterface& move_group);
    // 一步
    bool step(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                geometry_msgs::PoseStamped pose);
    bool swop(moveit::planning_interface::MoveGroupInterface& capture_move_group,\
                        moveit::planning_interface::MoveGroupInterface& rotate_move_group,\
                        geometry_msgs::PoseStamped pose);
    bool rotateCube(moveit::planning_interface::MoveGroupInterface& rotate_move_group, int angle);
    moveit::planning_interface::MoveItErrorCode setAndMove(moveit::planning_interface::MoveGroupInterface& move_group, \
                                                                        geometry_msgs::PoseStamped& );
    // 将面和角度映射到几号机器人抓住什么点位,并拧转多少度
    void analyseData(int face, int angle);


    // 記錄從 getPoseStamped() 獲取的點
    int  writePoseFile();
    // 現場獲取N個坐標
    void getPoseStamped();
    // 從文件讀取N個坐標
    inline void loadRobotPoses();
    void loadRobotPose(std::string path, int row, int column);
    void loadPickPose();
    
    int action();

    void loadPoseData();

    void initPose();

    bool openGripper(moveit::planning_interface::MoveGroupInterface& move_group);
    bool closeGripper(moveit::planning_interface::MoveGroupInterface& move_group);

    void setJointAllConstraints(moveit::planning_interface::MoveGroupInterface& move_group);
    void clearConstraints(moveit::planning_interface::MoveGroupInterface& group);

    void goPreparePose();

    int photograph();

    void photographstepDoublePhoto(int photoNum, int capturePose, int talkPose);
    void shoot(int num=0);
    void stopMove();
    void spinOnce();
    void robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z);
    moveit::planning_interface::MoveGroupInterface& getMoveGroup(int num);
    int placeCube();
    bool recordPose(int robotNum, std::string uri, std::string name, bool isJointSpce=false);
    int backHome(int robot);
    bool setRobotEnable();
    int moveToPose();
    int Cartesian();
    int updataPointData();
    bool pickCube();
    bool setStartState(moveit::planning_interface::MoveGroupInterface& group);
    int dataManagerClient();
    int loadPose(std::string uri, std::string name, geometry_msgs::PoseStamped& pose);
    int requestData();
    int solve();
    int magicMoveToPointFSM(std::vector<std::string> cmd);
    void robotMoveCartesian(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z);
    bool setAndMoveMulti(moveit::planning_interface::MoveGroupInterface& group, \
                        std::vector<geometry_msgs::PoseStamped>& poses, int type,\
                        trajectory_msgs::JointTrajectory& tra, 
                        bool only_plan);
    bool RobotTrajectory(moveit::planning_interface::MoveGroupInterface& group, \
                        geometry_msgs::PoseStamped& TargetPose, \
                        moveit_msgs::RobotTrajectory& trajectory,\
                        robot_state::RobotState& r);
    bool RobotTrajectoryLine(moveit::planning_interface::MoveGroupInterface& group, \
                geometry_msgs::PoseStamped& TargetPose, 
                moveit_msgs::RobotTrajectory& trajectory, \
                robot_state::RobotState& r);
    bool seedTrajectory(trajectory_msgs::JointTrajectory& trajectory0, trajectory_msgs::JointTrajectory& trajectory1);
    void InitializationState();
private:
    void getPrepareSomeDistanceRobotPose();

    inline void getPrepareSomeDistance(std::vector<std::vector<geometry_msgs::PoseStamped> >& pose, int row, int column);
    bool InitializationStateAction();
    void transformData();

    // bool analyseCallBack(rubik_cube_solve::rubik_cube_solve_cmd::Request& req, rubik_cube_solve::rubik_cube_solve_cmd::Response& rep);
    bool rbRunCommand(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep);

    bool magicMoveToPointCallback(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep);
    bool magicStepMoveCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);
    // bool magicRecordPoseCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);

    bool placeCubeCallback(rb_msgAndSrv::rb_ArrayAndBool::Request& req, rb_msgAndSrv::rb_ArrayAndBool::Response& rep);

    void sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg);
    void rubikCubeSolveDataCallBack(const std_msgs::Int8MultiArrayConstPtr& msg);

    // ros::ServiceServer analyseCmd;
    ros::ServiceServer end_effector;
    ros::ServiceServer beginSolve;
    ros::ServiceServer placeCubeServer;

    ros::ServiceServer nextStep;
    ros::ServiceServer lineCartesian;
    ros::ServiceServer recordPoint;
    ros::ServiceServer record_pose;

    ros::ServiceServer magicMoveToPoint;
    ros::ServiceServer magicStepMove;
    // ros::ServiceServer magicRecordPose;

    ros::ServiceClient receiveSolve;
    ros::ServiceClient shootClient;
    ros::ServiceClient openGripper_client0;
    ros::ServiceClient closeGripper_client0;
    ros::ServiceClient openGripper_client1;
    ros::ServiceClient closeGripper_client1;

    /***** motion ***/
    ros::ServiceClient l_motionStart_client;
    ros::ServiceClient r_motionStart_client;

    ros::ServiceClient l_moveToSiglePose_client;
    ros::ServiceClient r_moveToSiglePose_client;

    ros::ServiceClient l_moveToMultiPose_client;
    ros::ServiceClient r_moveToMultiPose_client;

    ros::ServiceClient l_moveLine_client;
    ros::ServiceClient r_moveLine_client;

    ros::ServiceClient l_SigleAixs_client;
    ros::ServiceClient r_SigleAixs_client;
    /****/

    /** dm_bridge **/
    ros::ServiceClient loadPoseClient;
    ros::ServiceClient addPoseClient;
    ros::ServiceClient savePoseEnd;
    ros::ServiceClient loadJointPoseClient;
    /****/

    /** planning_bridge **/
    // ros::ServiceClient 
    /****/

    void initMotionClient();
    void start(int robotNum);



    std::vector<double> getRobotState(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::PoseStamped& poseStamped);
    bool setAndMoveClient(moveit::planning_interface::MoveGroupInterface& move_group, \
                        geometry_msgs::PoseStamped& poseStamped);
    bool robotMoveCartesianUnitClient(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z);
    bool setJoint6ValueClient(moveit::planning_interface::MoveGroupInterface& rotate_move_group, int angle);
    bool setAndMoveMultiClient(moveit::planning_interface::MoveGroupInterface& move_group, \
                        std::vector<geometry_msgs::PoseStamped>& poseStamped);

    
    bool recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep);

    int stepCnt = 0;
    /*****/

    ros::Subscriber stopMoveSub;
    ros::Subscriber rubikCubeSolveData_sub;
    
    ros::Publisher progressPub;


    std::vector<int> rubikCubeSolveData;
    std::vector<std::vector<int> > rubikCubeSolvetransformData;

    ros::NodeHandle nh;

    std::vector<geometry_msgs::PoseStamped> photographPose;
    moveit::planning_interface::MoveGroupInterface* move_group0;
    moveit::planning_interface::MoveGroupInterface* move_group1;

    std::vector<std::vector<geometry_msgs::PoseStamped> > robotPose;
    const int ROWS = 2;
    const int COLUMNS = 8;

    RecordPoseData data0;
    RecordPoseData data1;

    CubeState Cstate;
    ActionData Adata;
    recoredPoint recordPointData;
    double prepare_some_distance;
    const double rubikCubeAdd = 0.0095;
    const int UP = 1;
    const int down = 2;

    int runModel = 0;

    bool isStop;
    bool isBegingSolve = false;
    bool isPlaceCube = false;
    bool isSwopOver = false;
    bool isTest = false;
    bool placeCubeRobot;
    int overStepflag = 0x0000;
};