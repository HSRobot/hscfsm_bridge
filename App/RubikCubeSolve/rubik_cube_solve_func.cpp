#include <rubik_cube_solve_func.h>
#include <math.h>
RubikCubeSolve::RubikCubeSolve(ros::NodeHandle nodehandle)
{
    move_group0 = new moveit::planning_interface::MoveGroupInterface("arm0");
    move_group1 = new moveit::planning_interface::MoveGroupInterface("arm1");
    nh = nodehandle;
    nh.param("/rubik_cube_solve/prepare_some_distance", prepare_some_distance, 0.05);
    // 测试使用
    // analyseCmd = nh.advertiseService("analyse_rubik_cube_cmd", &RubikCubeSolve::analyseCallBack, this);
    beginSolve = nh.advertiseService("/MagicStepRunCommand", &RubikCubeSolve::rbRunCommand, this);
    // 放魔方
    placeCubeServer = nh.advertiseService("/placeMagicCube", &RubikCubeSolve::placeCubeCallback, this);
    // 去到解魔方的某个动作
    magicMoveToPoint = nh.advertiseService("/magic_move_to_point", &RubikCubeSolve::magicMoveToPointCallback, this);
    // 用于自动记录魔方动作
    magicStepMove = nh.advertiseService("/magic_step_move", &RubikCubeSolve::magicStepMoveCallback, this);
    // 更新坐标
    record_pose = nh.advertiseService("record_pose", &RubikCubeSolve::recordPoseCallBack, this);
    // magicRecordPose = nh.advertiseService("/magic_recordPose", &RubikCubeSolve::magicRecordPoseCallback, this);
    // 进度条
    progressPub = nh.advertise<std_msgs::Int8MultiArray>("progress_rbSolveMagic", 10);
    // 停止运动
    stopMoveSub = nh.subscribe("/stop_move", 1, &RubikCubeSolve::sotpMoveCallback, this);
    // 接受魔方解析的数据
    rubikCubeSolveData_sub = nh.subscribe("cube_solution", 100, &RubikCubeSolve::rubikCubeSolveDataCallBack, this);
    // 控制夹爪
    openGripper_client0 = nh.serviceClient<hirop_msgs::openGripper>("/UR51/openGripper");
    closeGripper_client0 = nh.serviceClient<hirop_msgs::closeGripper>("/UR51/closeGripper");
    openGripper_client1 = nh.serviceClient<hirop_msgs::openGripper>("/UR52/openGripper");
    closeGripper_client1 = nh.serviceClient<hirop_msgs::closeGripper>("/UR52/closeGripper");
    // 用于拍照
    shootClient = nh.serviceClient<cubeParse::TakePhoto>("get_cube");
    // 请求解魔方数据
    receiveSolve = nh.serviceClient<cubeParse::SolveCube>("solve_cube");
    dataManagerClient();

    initMotionClient();

    robotPose.resize(ROWS);
    robotPose[0].resize(COLUMNS);
    robotPose[1].resize(COLUMNS);

    // setJointAllConstraints(*move_group0);
    // setJointAllConstraints(*move_group1);
    ROS_INFO_STREAM(move_group0->getPathConstraints());
    ROS_INFO_STREAM(move_group1->getPathConstraints());

    double speed;
    nh.param("/rubik_cube_solve/speed", speed, 1.0);
    move_group0->setMaxVelocityScalingFactor(speed);
    move_group1->setMaxVelocityScalingFactor(speed);
    move_group0->setGoalPositionTolerance(0.0001);
    move_group1->setGoalPositionTolerance(0.0001);
    loadPickPose();
    initPose();
    Cstate.isFinish = false;
    isBegingSolve = false;
    isStop = false;
}

RubikCubeSolve::~RubikCubeSolve()
{
    delete move_group0;
    delete move_group1;
}

int RubikCubeSolve::dataManagerClient()
{
    loadPoseClient = nh.serviceClient<hirop_msgs::loadPoseData>("/load_pose_data");
    addPoseClient = nh.serviceClient<hirop_msgs::addJointPose>("/add_pose_data");
    savePoseEnd = nh.serviceClient<hirop_msgs::saveDataEnd>("/save_data_end");
    loadJointPoseClient = nh.serviceClient<hirop_msgs::loadJointsData>("/load_joint_data");
    return 0;
}

void RubikCubeSolve::initMotionClient()
{
    l_motionStart_client = nh.serviceClient<hirop_msgs::motionBridgeStart>("/hsr_left/motionBridgeStart");
    r_motionStart_client = nh.serviceClient<hirop_msgs::motionBridgeStart>("/hsr_right/motionBridgeStart");

    l_moveToSiglePose_client = nh.serviceClient<hirop_msgs::moveToSiglePose>("/hsr_left/moveToSiglePose");
    r_moveToSiglePose_client = nh.serviceClient<hirop_msgs::moveToSiglePose>("/hsr_right/moveToSiglePose");

    l_moveToMultiPose_client = nh.serviceClient<hirop_msgs::moveToMultiPose>("/hsr_left/moveToMultiPose");
    r_moveToMultiPose_client = nh.serviceClient<hirop_msgs::moveToMultiPose>("/hsr_right/moveToMultiPose");

    l_moveLine_client = nh.serviceClient<hirop_msgs::moveLine>("/hsr_left/moveLine");
    r_moveLine_client = nh.serviceClient<hirop_msgs::moveLine>("/hsr_right/moveLine");

    l_SigleAixs_client = nh.serviceClient<hirop_msgs::moveSigleAixs>("/hsr_left/SigleAixs");
    r_SigleAixs_client = nh.serviceClient<hirop_msgs::moveSigleAixs>("/hsr_right/SigleAixs");
}

std::vector<double> RubikCubeSolve::getRobotState(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::PoseStamped &poseStamped)
{
    setStartState(move_group);
    unsigned int attempts = 10;
    double timeout = 0.1;
    moveit::core::RobotModelConstPtr robotModel = move_group.getRobotModel();
    moveit::core::RobotStatePtr robotState = move_group.getCurrentState();
    const robot_state::JointModelGroup *jointModelGroup = robotModel->getJointModelGroup(move_group.getName());
    std::vector<double> joints;
    bool flag;
    for (int i = 0; i < 10; i++)
    {
        flag = robotState->setFromIK(jointModelGroup, poseStamped.pose, attempts, timeout);
        if (flag)
        {
            robotState->copyJointGroupPositions(jointModelGroup, joints);
            if (move_group.getName() == "arm0")
            {
                if (joints[3] < -0.5)
                    continue;
                else if (joints[3] > 0.5)
                    continue;
            }
            ROS_INFO("IK SUCCEED");
            return joints;
        }
    }
    ROS_INFO("IK FAILED");
    return joints;
}

bool RubikCubeSolve::setAndMoveClient(moveit::planning_interface::MoveGroupInterface &move_group,
                                      geometry_msgs::PoseStamped &poseStamped)
{
    std::vector<double> joints;
    joints = getRobotState(move_group, poseStamped);
    if (joints.empty())
        return false;
    hirop_msgs::moveToSiglePose srv;
    srv.request.pose_joints_angle.joints_angle.data = joints;
    if (move_group.getName() == "arm0")
    {
        if (l_moveToSiglePose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if (r_moveToSiglePose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}

bool RubikCubeSolve::setAndMoveMultiClient(moveit::planning_interface::MoveGroupInterface &move_group,
                                           std::vector<geometry_msgs::PoseStamped> &poseStamped)
{
    hirop_msgs::moveToMultiPose srv;
    srv.request.poseList_joints_angle.resize(poseStamped.size());
    for (int i = 0; i < poseStamped.size(); i++)
    {
        std::vector<double> joint;
        joint = getRobotState(move_group, poseStamped[i]);
        if (joint.empty())
            return false;
        srv.request.poseList_joints_angle[i].joints_angle.data = joint;
    }
    if (move_group.getName() == "arm0")
    {
        if (l_moveToMultiPose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if (r_moveToMultiPose_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}

bool RubikCubeSolve::robotMoveCartesianUnitClient(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    hirop_msgs::moveLine srv;
    srv.request.Cartesian_x = x;
    srv.request.Cartesian_y = y;
    srv.request.Cartesian_z = z;
    if (group.getName() == "arm0")
    {
        if (l_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if (r_moveLine_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    return false;
}

bool RubikCubeSolve::setJoint6ValueClient(moveit::planning_interface::MoveGroupInterface &rotate_move_group, int angle)
{
    setStartState(rotate_move_group);
    std::vector<double> joint = rotate_move_group.getCurrentJointValues();
    double before = joint[5];
    double a = static_cast<double>(angle);
    a = angle2rad(a);
    joint[5] += a;

    if ((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 180 || angle == -180))
    {
        ROS_INFO_STREAM("joint_6:" << joint[5]);
        joint[5] -= 2 * a;
    }
    else if ((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 90 || angle == -90))
    {
        joint[5] -= 4 * a;
    }
    hirop_msgs::moveSigleAixs srv;
    // joint[5] /= M_PI;
    // joint[5] *= 180;
    srv.request.angle = joint[5];
    srv.request.index_axis = 5;
    if (rotate_move_group.getName() == "arm0")
    {
        if (l_SigleAixs_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    else
    {
        if (r_SigleAixs_client.call(srv))
        {
            ROS_INFO(srv.response.info.c_str());
            return srv.response.is_success;
        }
    }
    setStartState(rotate_move_group);
    std::vector<double> j = rotate_move_group.getCurrentJointValues();
    return false;
}

/****/

bool RubikCubeSolve::magicMoveToPointCallback(rb_msgAndSrv::rb_ArrayAndBool::Request &req, rb_msgAndSrv::rb_ArrayAndBool::Response &rep)
{
    recordPointData.model = req.data[0];
    recordPointData.stepNum = req.data[1];
    ROS_INFO_STREAM(recordPointData.model << recordPointData.stepNum);
    moveToPose();
    rep.respond = true;
    return true;
}

int RubikCubeSolve::magicMoveToPointFSM(std::vector<std::string> cmd)
{
    recordPointData.model = std::atoi(cmd[0].c_str());
    recordPointData.stepNum = std::atoi(cmd[1].c_str());
    ROS_INFO_STREAM("model: " << recordPointData.model << " step: " << recordPointData.stepNum);
    moveToPose();
    return 0;
}

bool RubikCubeSolve::magicStepMoveCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
{
    Cartesian();
    return true;
}

// bool RubikCubeSolve::magicRecordPoseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rep)
// {
//     updataPointData();
//     return true;
// }

bool RubikCubeSolve::placeCubeCallback(rb_msgAndSrv::rb_ArrayAndBool::Request &req, rb_msgAndSrv::rb_ArrayAndBool::Response &rep)
{
    stopMove();
    placeCubeRobot = req.data[0];
    ROS_INFO_STREAM(placeCubeRobot);
    if (isBegingSolve || isTest)
        isPlaceCube = true;
    else
    {
        placeCube();
    }
    return true;
}

bool RubikCubeSolve::rbRunCommand(rb_msgAndSrv::rb_ArrayAndBool::Request &req, rb_msgAndSrv::rb_ArrayAndBool::Response &rep)
{
    nh.setParam("/isRuning_solveMagic", true);
    if (isBegingSolve != true)
    {
        isBegingSolve = true;
    }
    runModel = req.data[0];
    isStop = false;
    spinOnce();
    rep.respond = true;
    isStop = false;
    return true;
}

void RubikCubeSolve::rubikCubeSolveDataCallBack(const std_msgs::Int8MultiArrayConstPtr &msg)
{
    std::vector<int>().swap(rubikCubeSolveData);
    rubikCubeSolveData.resize(msg->data.size());
    for (int i = 0; i < msg->data.size(); i++)
    {
        // rubikCubeSolveData.push_back(msg->data[i]);
        rubikCubeSolveData[i] = msg->data[i];
    }
    Cstate.isFinish = false;
    transformData();
    ROS_INFO_STREAM("catch data");
}

void RubikCubeSolve::transformData()
{
    int angleFlag;
    int angle;
    int face;
    rubikCubeSolvetransformData.resize(rubikCubeSolveData.size());
    for (int i = 0; i < rubikCubeSolveData.size(); ++i)
    {
        rubikCubeSolvetransformData[i].resize(2);
        angleFlag = rubikCubeSolveData[i] % 3;
        if (angleFlag == 0)
            rubikCubeSolvetransformData[i][1] = 90;
        else if (angleFlag == 1)
            rubikCubeSolvetransformData[i][1] = 180;
        else
            rubikCubeSolvetransformData[i][1] = -90;
        if (rubikCubeSolveData[i] >= 0 && rubikCubeSolveData[i] <= 2)
        {
            face = 6;
        }
        else if (rubikCubeSolveData[i] >= 3 && rubikCubeSolveData[i] <= 5)
        {
            face = 5;
        }
        else if (rubikCubeSolveData[i] >= 6 && rubikCubeSolveData[i] <= 8)
        {
            face = 4;
        }
        else if (rubikCubeSolveData[i] >= 9 && rubikCubeSolveData[i] <= 11)
        {
            face = 3;
        }
        else if (rubikCubeSolveData[i] >= 12 && rubikCubeSolveData[i] <= 14)
        {
            face = 1;
        }
        else
        {
            face = 2;
        }
        rubikCubeSolvetransformData[i][0] = face;
        ROS_INFO_STREAM("face: " << rubikCubeSolvetransformData[i][0] << ", angle: " << rubikCubeSolvetransformData[i][1]);
    }
}

void RubikCubeSolve::stopMove()
{
    move_group0->stop();
    move_group1->stop();
    isStop = true;
}

void RubikCubeSolve::goPreparePose()
{
    setAndMove(*move_group1, robotPose[1][0]);
    setAndMove(*move_group0, robotPose[0][0]);
    InitializationStateAction();
}

void RubikCubeSolve::photographstepDoublePhoto(int photoNum, int capturePose, int talkPose)
{
    if (!isStop)
    {
        setAndMove(getMoveGroup(Adata.captureRobot), robotPose[Adata.captureRobot][3]);

        setAndMove(getMoveGroup(Adata.captureRobot), photographPose[capturePose]);

        setAndMove(getMoveGroup(Adata.otherRobot), photographPose[talkPose]);

        shoot(photoNum);
        setJoint6Value(getMoveGroup(Adata.captureRobot), 180);
        shoot(++photoNum);
        setAndMove(getMoveGroup(Adata.otherRobot), robotPose[Adata.otherRobot][0]);

        setAndMove(getMoveGroup(Adata.captureRobot), robotPose[Adata.captureRobot][3]);
        geometry_msgs::PoseStamped pose = robotPose[Adata.captureRobot][Adata.capturePoint];
        pose.pose.position.y += pow(-1, Adata.captureRobot) * prepare_some_distance;
        setAndMove(getMoveGroup(Adata.captureRobot), pose);
    }
}

void RubikCubeSolve::shoot(int num)
{
    if (!isStop)
    {
        ROS_INFO_STREAM("shoot" << num);
        cubeParse::TakePhoto srv;
        srv.request.photoNum = num;
        shootClient.call(srv);
        ros::WallDuration(2.0).sleep();
    }
}

// 拍照点位
int RubikCubeSolve::photograph()
{
    backHome(1);
    // 抓起魔方
    // PosepickPose0.yaml
    pickCube();
    goPreparePose();
    // PosepickPose1.yaml
    setAndMove(*move_group1, photographPose[1]);
    // PosepickPose2.yaml
    setAndMove(*move_group0, photographPose[2]);
    shoot(0);

    setAndMove(*move_group1, robotPose[1][0]);
    setAndMove(*move_group0, robotPose[0][0]);

    // robot1 PosepickPose3.yaml; robot0 PosepickPose4.yaml
    photographstepDoublePhoto(1, 3, 4);

    analyseData(3, 0);
    swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);

    // robot0 PosepickPose5.yaml
    setAndMove(*move_group0, photographPose[5]);
    // robot1 PosepickPose6.yaml
    setAndMove(*move_group1, photographPose[6]);
    shoot(3);

    setAndMove(*move_group1, robotPose[1][0]);
    setAndMove(*move_group0, robotPose[0][0]);
    // robot0 PosepickPose3.yam7; robot1 PosepickPose8.yaml
    photographstepDoublePhoto(4, 7, 8);
    return 0;
}

bool RubikCubeSolve::setRobotEnable()
{
    system("rosrun rubik_cube_solve set_robot_enable_false.sh");
    system("rosrun rubik_cube_solve set_robot_enable_true.sh");
    return true;
}

int RubikCubeSolve::placeCube()
{
    isStop = false;
    int robotNum = placeCubeRobot;
    backHome(0);
    backHome(1);
    tf2::Quaternion orientation;
    geometry_msgs::PoseStamped placeCubePose;
    ROS_INFO_STREAM("robotNum: " << robotNum);
    if (robotNum == 0)
    {
        placeCubePose = photographPose[9];
    }
    else
    {
        placeCubePose = photographPose[0];
    }
    setAndMove(getMoveGroup(robotNum), placeCubePose);
    robotMoveCartesian(getMoveGroup(robotNum), 0, 0, -prepare_some_distance);
    openGripper(getMoveGroup(robotNum));
    robotMoveCartesian(getMoveGroup(robotNum), 0, 0, prepare_some_distance);
    isPlaceCube = false;
    nh.setParam("/isRuning_solveMagic", false);
    return 0;
}

void RubikCubeSolve::InitializationState()
{
    Cstate.captureRobot = 1;
    Cstate.capturePoint = 3;
    Cstate.canRotateFace1 = 5;
    Cstate.canRotateFace2 = 2;
    Adata.captureRobot = Cstate.captureRobot;
    Adata.capturePoint = Cstate.capturePoint;
    Adata.otherRobot = (Cstate.captureRobot + 1) % 2;
}

bool RubikCubeSolve::InitializationStateAction()
{
    geometry_msgs::PoseStamped pose = robotPose[Cstate.captureRobot][Cstate.capturePoint];
    pose.pose.position.y += pow(-1, Cstate.captureRobot) * prepare_some_distance;
    setAndMove(getMoveGroup(Cstate.captureRobot), pose);
    setAndMove(getMoveGroup(Adata.otherRobot), robotPose[Adata.otherRobot][0]);
}

void RubikCubeSolve::setJointAllConstraints(moveit::planning_interface::MoveGroupInterface &move_group)
{
    moveit_msgs::JointConstraint jointCon;
    moveit_msgs::Constraints con;
    // set 6 joint
    std::vector<double> joint = move_group.getCurrentJointValues();
    if (move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_6";
    }
    else
    {
        jointCon.joint_name = "R_joint_6";
    }
    jointCon.position = joint[5];
    jointCon.tolerance_above = 4.8844;
    jointCon.tolerance_below = 4.8844;
    jointCon.weight = 0.9;
    con.joint_constraints.push_back(jointCon);

    // 4 joint
    if (move_group.getName() == "arm0")
    {
        jointCon.joint_name = "joint_4";
        jointCon.position = 1.0;
        jointCon.tolerance_above = 2;
        jointCon.tolerance_below = 1.5;
    }
    else
    {
        jointCon.joint_name = "R_joint_4";
        jointCon.position = -1.0;
        jointCon.tolerance_above = 1.5;
        jointCon.tolerance_below = 2.0;
    }
    jointCon.weight = 1;

    con.joint_constraints.push_back(jointCon);
    move_group.setPathConstraints(con);
    move_group.setPlanningTime(1.0);
}

void RubikCubeSolve::clearConstraints(moveit::planning_interface::MoveGroupInterface &move_group)
{
    move_group.clearPathConstraints();
    move_group.clearTrajectoryConstraints();
}

moveit::planning_interface::MoveGroupInterface &RubikCubeSolve::getMoveGroup(int num)
{
    if (num == 0)
        return *move_group0;
    else if (num == 1)
        return *move_group1;
}

// bool RubikCubeSolve::analyseCallBack(rubik_cube_solve::rubik_cube_solve_cmd::Request &req, rubik_cube_solve::rubik_cube_solve_cmd::Response &rep)
// {
//     isTest = true;
//     static int cnt = 0;
//     int flag;
//     nh.getParam("/rubik_cube_solve/test", flag);
//     if (req.face == 0)
//     {
//         cnt++;
//         if (flag == 0)
//         {
//             // 測試拍照
//             ROS_INFO_STREAM("test 0");
//             photograph();
//         }
//         else if (flag == 1)
//         {
//             // 去到預備動作
//             ROS_INFO_STREAM("test 1");
//             InitializationState();
//             goPreparePose();
//         }
//         else if (flag == 2)
//         {
//         }
//         else if (flag == 3)
//         {
//             // 测试魔方解算数据
//             backHome(1);
//             pickCube();
//             cubeParse::SolveCube srv;
//             receiveSolve.call(srv);
//             goPreparePose();
//             int cnt = 1;
//             for (auto it : rubikCubeSolvetransformData)
//             {
//                 ROS_INFO_STREAM("step: " << cnt << " " << it[0] << " " << it[1]);
//                 cnt++;
//             }
//             cnt = 1;
//             for (int i = 0; i < rubikCubeSolvetransformData.size() && ros::ok() && !isStop; ++i)
//             {
//                 ROS_INFO("step: %d, count: %d", cnt, rubikCubeSolvetransformData.size());
//                 cnt++;
//                 analyseData(rubikCubeSolvetransformData[i][0], rubikCubeSolvetransformData[i][1]);
//                 action();
//             }
//             std::vector<std::vector<int>>().swap(rubikCubeSolvetransformData);
//             Cstate.isFinish = true;
//             placeCube();
//             isBegingSolve = false;
//         }
//         else if (flag == 4)
//         {
//             // 測試機器人1的精度
//             ROS_INFO_STREAM("test 4");
//             backHome(1);
//             setAndMove(*move_group1, photographPose[0]);
//             openGripper(*move_group1);
//             robotMoveCartesian(*move_group1, 0, 0, -prepare_some_distance);
//         }
//         else if (flag == 5)
//         {
//             // 放置魔方
//             ROS_INFO_STREAM("test 5");
//             InitializationState();
//             goPreparePose();
//             placeCube();
//         }
//         else if (flag == 6)
//         {
//             // 測試拿起魔方的動作
//         }
//         else if (flag == 7)
//         {
//             // 測試拿起魔方的動作
//         }
//         else if (flag == 8)
//         {
//             // 測試機器人1的精度
//             ROS_INFO_STREAM("test 8");
//             backHome(1);
//             pickCube();
//             goPreparePose();
//         }
//         else if (flag == 9)
//         {
//             ROS_INFO_STREAM("test 9");
//             photograph();
//         }
//     }
//     else
//     {
//         if (cnt == 0)
//         {
//             InitializationState();
//             goPreparePose();
//         }
//         analyseData(req.face, req.angle);
//         action();
//     }
//     rep.isFinish = true;
//     if (isPlaceCube)
//     {
//         placeCube();
//     }
//     isTest = false;
//     return true;
// }

bool RubikCubeSolve::setStartState(moveit::planning_interface::MoveGroupInterface &group)
{
    group.setStartStateToCurrentState();
    return true;
}

bool RubikCubeSolve::recordPose(int robotNum, std::string uri, std::string name, bool isJointSpce)
{
    setStartState(getMoveGroup(robotNum));
    std::string path;
    path = "rubik_cube/";
    path = path + uri;
    if (isJointSpce)
    {
        ROS_ERROR("record joint space in development ...");
    }
    else
    {
        geometry_msgs::PoseStamped pose;
        getMoveGroup(robotNum).getCurrentPose();
        pose = getMoveGroup(robotNum).getCurrentPose();
        ROS_INFO_STREAM(pose);
        hirop_msgs::savePoseData srv;
        srv.request.pose = pose;
        addPoseClient.call(srv);
    }
    hirop_msgs::saveDataEnd srvPath;
    srvPath.request.uri = path;
    srvPath.request.name = name;
    savePoseEnd.call(srvPath);
    return true;
}

void RubikCubeSolve::sotpMoveCallback(const std_msgs::Bool::ConstPtr &msg)
{
    stopMove();
}

void RubikCubeSolve::analyseData(int face, int angle)
{

    if (face == Cstate.canRotateFace1)
    {
        Adata.space = 1;
        Adata.isSwop = false;
    }
    else if (face == Cstate.canRotateFace2)
    {
        Adata.space = 2;
        Adata.isSwop = false;
    }
    else
    {
        switch (face)
        {
        case 1:
            Adata.capturePoint = (Cstate.capturePoint + 1) % 4 + 1;
            Adata.space = 2;
            Cstate.canRotateFace1 = Adata.capturePoint + 2;
            Cstate.canRotateFace2 = 1;
            break;
        case 2:
            Adata.capturePoint = (Cstate.capturePoint + 1) % 4 + 1;
            Adata.space = 2;
            Cstate.canRotateFace1 = Adata.capturePoint + 2;
            Cstate.canRotateFace2 = 2;
            break;
        case 3:
            Adata.capturePoint = 1;
            Adata.space = 1;
            Cstate.canRotateFace1 = 3;
            Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
            break;
        case 4:
            Adata.capturePoint = 2;
            Adata.space = 1;
            Cstate.canRotateFace1 = 4;
            Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
            break;
        case 5:
            Adata.capturePoint = 3;
            Adata.space = 1;
            Cstate.canRotateFace1 = 5;
            Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
            break;
        case 6:
            Adata.capturePoint = 4;
            Adata.space = 1;
            Cstate.canRotateFace1 = 6;
            Cstate.canRotateFace2 = Cstate.canRotateFace2 % 2 + 1;
            break;
        }
        Adata.isSwop = true;
        isSwopOver = false;
        Adata.captureRobot = (Cstate.captureRobot + 1) % 2;
        // Cstate.captureRobot = Adata.captureRobot;
        // Cstate.capturePoint = Adata.capturePoint;
    }
    Adata.angle = angle;
    Adata.otherRobot = (Adata.captureRobot + 1) % 2;
}

int RubikCubeSolve::requestData()
{
    cubeParse::SolveCube srv;
    if (receiveSolve.call(srv))
        return 0;
    else
        return -1;
}

int RubikCubeSolve::solve()
{
    isStop = false;
    ROS_INFO("begin solve .....");
    int cnt = 1;
    std_msgs::Int8MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = rubikCubeSolvetransformData.size();
    for (int i = 0; i < rubikCubeSolvetransformData.size() && ros::ok() && !isStop; ++i)
    {
        ROS_INFO("begin step: %d, count: %d", cnt, rubikCubeSolvetransformData.size());
        analyseData(rubikCubeSolvetransformData[i][0], rubikCubeSolvetransformData[i][1]);
        action();
        cnt++;
        msg.data[1] = i + 1;
        progressPub.publish(msg);
        if (cnt % 10 == 0)
        {
            setRobotEnable();
        }
    }
    std::vector<std::vector<int>>().swap(rubikCubeSolvetransformData);
    Cstate.isFinish = true;
    return 0;
}

void RubikCubeSolve::spinOnce()
{

    if (isBegingSolve)
    {
        if (runModel == 1 || runModel == 4)
        {
            ROS_INFO("start");
            photograph();
            setRobotEnable();
            ros::Duration(1.0).sleep();
        }
        if (runModel == 2 || runModel == 4)
        {
            ROS_INFO("get solve data");
            requestData();
        }
        if (runModel == 3 || runModel == 4)
        {
            solve();
        }
        if (Cstate.isFinish || isPlaceCube)
        {
            if (Cstate.isFinish)
                placeCubeRobot = Cstate.captureRobot;
            placeCube();
        }
        Cstate.isFinish = false;
        isBegingSolve = false;
        runModel = 0;
    }
}

int RubikCubeSolve::action()
{
    if (Adata.isSwop)
    {
        // 拧魔方
        swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
    }
    // MOVE TO THE TARGET
    step(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
    return 0;
}

//放置 魔方
bool RubikCubeSolve::swop(moveit::planning_interface::MoveGroupInterface &capture_move_group,
                          moveit::planning_interface::MoveGroupInterface &rotate_move_group,
                          geometry_msgs::PoseStamped pose)
{
    std::vector<trajectory_msgs::JointTrajectory> tra;
    tra.resize(2);
    openGripper(capture_move_group);

    geometry_msgs::PoseStamped targetPose;
    std::vector<geometry_msgs::PoseStamped> wayPoint;
    wayPoint.push_back(pose);
    targetPose = pose;
    targetPose.pose.position.y += pow(-1, Adata.captureRobot) * prepare_some_distance;
    wayPoint.push_back(targetPose);
    // 获取轨迹
    setAndMoveMulti(capture_move_group, wayPoint, 0, tra[Adata.captureRobot], false);

    closeGripper(capture_move_group);

    openGripper(rotate_move_group);
    isSwopOver = true;
    Cstate.captureRobot = Adata.captureRobot;
    Cstate.capturePoint = Adata.capturePoint;

    std::vector<geometry_msgs::PoseStamped> wayPoint2;
    setStartState(rotate_move_group);
    geometry_msgs::PoseStamped targetPose2 = rotate_move_group.getCurrentPose();
    targetPose2.pose.position.y += pow(-1, Adata.captureRobot) * prepare_some_distance;
    wayPoint2.push_back(targetPose2);
    wayPoint2.push_back(robotPose[Adata.otherRobot][0]);
    // 获取轨迹
    setAndMoveMulti(rotate_move_group, wayPoint2, 1, tra[Adata.otherRobot], false);
    return true;
}

// step 魔方提起
bool RubikCubeSolve::step(moveit::planning_interface::MoveGroupInterface &capture_move_group,
                          moveit::planning_interface::MoveGroupInterface &rotate_move_group,
                          geometry_msgs::PoseStamped pose)
{
    std::vector<trajectory_msgs::JointTrajectory> tra;
    tra.resize(2);
    std::vector<geometry_msgs::PoseStamped> wayPoints;
    std::vector<geometry_msgs::PoseStamped> wayPoints4;
    if (Adata.space == UP)
    {
        wayPoints.push_back(robotPose[Adata.captureRobot][3]);
        wayPoints.push_back(robotPose[Adata.captureRobot][6]);
        // 获取发送
        setAndMoveMulti(capture_move_group, wayPoints, 2, tra[Adata.captureRobot], true);
        wayPoints4.push_back(robotPose[Adata.otherRobot][7]);
    }
    else
    {
        wayPoints4.push_back(robotPose[Adata.otherRobot][5]);
    }
    setAndMoveMulti(rotate_move_group, wayPoints4, 2, tra[Adata.otherRobot], true);
    seedTrajectory(tra[0], tra[1]);
    // 去 轉動
    rotateCube(rotate_move_group, Adata.angle);
    // 拧魔方的回原位
    std::vector<trajectory_msgs::JointTrajectory> tra2;
    tra2.resize(2);

    geometry_msgs::PoseStamped targetPose;
    std::vector<geometry_msgs::PoseStamped> wayPoints2;
    setStartState(rotate_move_group);
    targetPose = rotate_move_group.getCurrentPose();
    if (Adata.space == UP)
    {
        // targetPose.pose.position.y -= pow(-1, Adata.otherRobot)*prepare_some_distance*0.7071067;
        // targetPose.pose.position.z -= 0.7071067*prepare_some_distance;
        robotMoveCartesian(rotate_move_group, 0, -pow(-1, Adata.otherRobot) * prepare_some_distance * 0.7071067, -0.7071067 * prepare_some_distance);
    }
    else
    {
        robotMoveCartesian(rotate_move_group, 0, -pow(-1, Adata.otherRobot) * prepare_some_distance, 0);
        // targetPose.pose.position.y -= pow(-1, Adata.otherRobot)*prepare_some_distance;
    }

    // wayPoints2.push_back(targetPose);
    wayPoints2.push_back(robotPose[Adata.otherRobot][0]);
    setAndMoveMulti(rotate_move_group, wayPoints2, 2, tra2[Adata.otherRobot], true);
    // 抓住魔方的回原位
    pose.pose.position.y += pow(-1, Adata.captureRobot) * prepare_some_distance;

    std::vector<geometry_msgs::PoseStamped> wayPoints3;
    if (Adata.space == UP)
    {
        wayPoints3.push_back(robotPose[Adata.captureRobot][3]);
        wayPoints3.push_back(pose);
        setAndMoveMulti(capture_move_group, wayPoints3, 2, tra2[Adata.captureRobot], true);
    }
    seedTrajectory(tra2[0], tra2[1]);
    return true;
}

//TODO rotateCube
bool RubikCubeSolve::rotateCube(moveit::planning_interface::MoveGroupInterface &rotate_move_group, int angle)
{
    openGripper(rotate_move_group);
    if (Adata.space == UP)
    {
        robotMoveCartesian(rotate_move_group, 0, pow(-1, Adata.otherRobot) * prepare_some_distance * 0.7071067, 0.7071067 * prepare_some_distance);
    }
    else
    {
        robotMoveCartesian(rotate_move_group, 0, pow(-1, Adata.otherRobot) * prepare_some_distance * 0.7071067, 0);
    }
    closeGripper(rotate_move_group);
    setJoint6Value(rotate_move_group, angle);
    openGripper(rotate_move_group);
    return true;
}

bool RubikCubeSolve::openGripper(moveit::planning_interface::MoveGroupInterface &move_group)
{
    bool flag = true;
    hirop_msgs::openGripper srv;
    if (!isStop)
    {
        if (move_group.getName() == "arm0")
        {
            flag = openGripper_client0.call(srv);
            ROS_INFO("arm0 openGripper ");
        }
        else
        {
            ROS_INFO("arm1 openGripper ");
            flag = openGripper_client1.call(srv);
        }
        ros::Duration(1).sleep();
    }
    return flag;
}
bool RubikCubeSolve::closeGripper(moveit::planning_interface::MoveGroupInterface &move_group)
{
    bool flag = true;
    hirop_msgs::closeGripper srv;
    if (!isStop)
    {
        if (move_group.getName() == "arm0")
        {
            ROS_INFO("arm0 closeGripper ");
            flag = closeGripper_client0.call(srv);
        }
        else
        {
            ROS_INFO("arm1 closeGripper ");
            flag = closeGripper_client1.call(srv);
        }
        ros::Duration(1).sleep();
    }
    return flag;
}

void RubikCubeSolve::initPose()
{
    this->loadRobotPoses();
    getPrepareSomeDistanceRobotPose();
}

void RubikCubeSolve::getPrepareSomeDistanceRobotPose()
{
    const double cos45 = 0.7071067;
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLUMNS; ++j)
        {
            if (j == 7)
            {
                robotPose[i][j].pose.position.z -= cos45 * prepare_some_distance;
                robotPose[i][j].pose.position.y -= pow(-1, i) * prepare_some_distance * cos45;
                continue;
            }
            getPrepareSomeDistance(robotPose, i, j);
        }
}

inline void RubikCubeSolve::getPrepareSomeDistance(std::vector<std::vector<geometry_msgs::PoseStamped>> &pose, int row, int column)
{
    if (column != 0 && column != 6)
        pose[row][column].pose.position.y -= pow(-1, row) * prepare_some_distance;
}

int RubikCubeSolve::loadPose(std::string uri, std::string name, geometry_msgs::PoseStamped &pose)
{
    hirop_msgs::loadPoseData srv;
    std::string uri_prefix = "/rubik_cube/";
    uri = uri_prefix + uri;
    srv.request.uri = uri;
    srv.request.name = name;
    if (loadPoseClient.call(srv))
    {
        ROS_INFO_STREAM("load pose " << name << " SUCCEED");
        pose = srv.response.poses[0];
        return 0;
    }
    else
    {
        ROS_INFO_STREAM("load pose " << name << " FAILED");
    }
    return -1;
}

void RubikCubeSolve::RubikCubeSolve::loadRobotPoses()
{
    std::vector<std::string> solvePoseName = {"pose03", "pose021", "pose022", "pose023",
                                              "pose024", "pose025", "pose011", "pose012",
                                              "pose14", "pose121", "pose122", "pose123",
                                              "pose124", "pose125", "pose111", "pose112"};
    std::size_t cnt = 0;
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            loadPose("pose", solvePoseName[cnt], robotPose[i][j]);
            cnt++;
        }
    }
}

void RubikCubeSolve::loadPickPose()
{
    std::string path;
    YAML::Node doc;
    std::vector<std::string> pickPoseName = {"Pose0", "Pose1", "Pose2",
                                             "Pose3", "Pose4", "Pose5",
                                             "Pose6", "Pose7", "Pose8", "Pose9"};
    photographPose.resize(pickPoseName.size());
    for (int i = 0; i < pickPoseName.size(); ++i)
    {
        loadPose("shootPose", pickPoseName[i], photographPose[i]);
        if (i == 0 || i == 9)
        {
            photographPose[i].pose.position.z += prepare_some_distance;
        }
    }
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::setAndMove(moveit::planning_interface::MoveGroupInterface &move_group,
                                                                       geometry_msgs::PoseStamped &poseStamped)
{
    // if(move_group.getName() == "arm1")
    //     ros::Duration(1.0).sleep();
    setJointAllConstraints(move_group);
    std::vector<double> joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(poseStamped);
    moveit::planning_interface::MoveItErrorCode code;
    code = this->moveGroupPlanAndMove(move_group, my_plan);
    clearConstraints(move_group);
    return code;
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::moveGroupPlanAndMove(moveit::planning_interface::MoveGroupInterface &move_group,
                                                                                 moveit::planning_interface::MoveGroupInterface::Plan &my_plan)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    if (!isStop)
        do
        {
            code = move_group.plan(my_plan);
        } while (ros::ok() && cnt < 10 && code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop);
    if (code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop)
    {
        code = loop_move(move_group);
    }
    return code;
}

moveit::planning_interface::MoveItErrorCode RubikCubeSolve::loop_move(moveit::planning_interface::MoveGroupInterface &move_group)
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        ROS_INFO_STREAM("loop_move");
        code = move_group.move();
        cnt++;
    } while (ros::ok() && cnt < 10 && code.val == moveit::planning_interface::MoveItErrorCode::TIMED_OUT && !isStop);
    return code;
}

inline double RubikCubeSolve::angle2rad(double &angle)
{
    return (angle / 180) * M_PI;
}

void RubikCubeSolve::setJoint6Value(moveit::planning_interface::MoveGroupInterface &move_group, int angle, bool joint_mode)
{
    double a = static_cast<double>(angle);
    std::vector<double> joint;

    joint = move_group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    move_group.setStartState(r);
    move_group.setStartStateToCurrentState();
    ROS_INFO_STREAM("joint_6 current:" << joint[5]);

    a = angle2rad(a);
    joint[5] += a;

    if ((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 180 || angle == -180))
    {
        ROS_INFO_STREAM("joint_6:" << joint[5]);
        joint[5] -= 2 * a;
    }
    else if ((joint[5] > 5.0 || joint[5] < -5.0) && (angle == 90 || angle == -90))
    {
        joint[5] -= 4 * a;
    }
    ROS_INFO_STREAM("joint_6 reference:" << joint[5]);
    move_group.setJointValueTarget(joint);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    while (ros::ok() && !isStop)
    {
        if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            move_group.execute(my_plan);
            break;
        }
    }
}

int RubikCubeSolve::writePoseFile()
{
    std::string path;
    std::vector<std::string> solvePoseName = {"pose03", "pose021", "pose022", "pose023",
                                              "pose024", "pose025", "pose011", "pose012",
                                              "pose14", "pose121", "pose122", "pose123",
                                              "pose124", "pose125", "pose111", "pose112"};
    std::size_t cnt = 0;
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < 8; j++)
        {
            recordPose(i, "pose", solvePoseName[cnt], false);
            cnt++;
        }
    }
}


bool RubikCubeSolve::recordPoseCallBack(rubik_cube_solve::recordPoseStamped::Request& req, rubik_cube_solve::recordPoseStamped::Response& rep)
{
    rep.isFinish = true;
    return true;
}



void RubikCubeSolve::robotMoveCartesianUnit2(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    // if(group.getName() == "arm1")
    ros::Duration(1).sleep();
    std::vector<double> joint = group.getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    group.setStartState(r);
    group.setStartStateToCurrentState();
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x += x;
    temp_pose.pose.position.y += y;
    temp_pose.pose.position.z += z;
    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    while (group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1.0 && !isStop && ros::ok())
        ;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (!isStop)
        group.execute(plan);
}

int RubikCubeSolve::backHome(int robot)
{
    setStartState(getMoveGroup(robot));
    if (!isStop)
    {
        ROS_INFO_STREAM("back home");
        const std::string home = "home" + std::to_string(robot);
        getMoveGroup(robot).setNamedTarget(home);
        getMoveGroup(robot).move();
    }
    return 0;
}

bool RubikCubeSolve::pickCube()
{
    setAndMove(*move_group1, photographPose[0]);
    openGripper(*move_group1);
    robotMoveCartesian(*move_group1, 0, 0, -prepare_some_distance);
    closeGripper(*move_group1);
    InitializationState();
    robotMoveCartesian(*move_group1, 0, 0, prepare_some_distance);
    return true;
}

int RubikCubeSolve::moveToPose()
{
    // system("rosrun rubik_cube_solve set_robot_enable_true.sh");
    // 复原魔方
    if (recordPointData.stepNum == 0)
    {
        overStepflag = 0;
        setAndMove(*move_group1, photographPose[0]);
        openGripper(*move_group1);
        InitializationState();
    }
    // 1 + 17 = 18
    if (recordPointData.model == 0)
    {
        switch (recordPointData.stepNum)
        {
        case 1:
            // robot1
            closeGripper(*move_group1);
            robotMoveCartesian(*move_group1, 0, 0, prepare_some_distance);
            setAndMove(*move_group1, robotPose[1][0]);
            break;
        case 2:
            // robot0
            setAndMove(*move_group0, robotPose[0][0]);
            break;
        case 3:
            // robot1
            setAndMove(*move_group1, robotPose[1][3]);
            robotMoveCartesian(*move_group1, 0, -prepare_some_distance, 0);
            break;
        case 4:
            setAndMove(*move_group0, robotPose[0][1]);
            break;
        case 5:
            robotMoveCartesian(*move_group0, 0, -prepare_some_distance, 0);
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group0, robotPose[0][2]);
            break;
        case 6:
            robotMoveCartesian(*move_group0, 0, -prepare_some_distance, 0);
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group0, robotPose[0][4]);
            break;
        case 7:
            robotMoveCartesian(*move_group0, 0, -prepare_some_distance, 0);
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group0, robotPose[0][5]);
            break;
        case 8:
            robotMoveCartesian(*move_group0, 0, -prepare_some_distance, 0);
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group1, robotPose[1][6]);
            break;
        case 9:
            setAndMove(*move_group0, robotPose[0][7]);
            break;
        case 10:
            robotMoveCartesian(*move_group0, 0, -prepare_some_distance * 0.707, -prepare_some_distance * 0.707);
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group1, robotPose[1][3]);
            robotMoveCartesian(*move_group1, 0, -prepare_some_distance, 0);
            setAndMove(*move_group0, robotPose[0][3]);
            break;
        case 11:
            closeGripper(*move_group0);
            openGripper(*move_group1);
            robotMoveCartesian(*move_group1, 0, prepare_some_distance, 0);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group1, robotPose[1][1]);
            break;
        case 12:
            robotMoveCartesian(*move_group1, 0, prepare_some_distance, 0);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group1, robotPose[1][2]);
            break;
        case 13:
            robotMoveCartesian(*move_group1, 0, prepare_some_distance, 0);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group1, robotPose[1][4]);
            break;
        case 14:
            robotMoveCartesian(*move_group1, 0, prepare_some_distance, 0);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group1, robotPose[1][5]);
            break;
        case 15:
            robotMoveCartesian(*move_group1, 0, prepare_some_distance, 0);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group0, robotPose[0][6]);
            break;
        case 16:
            setAndMove(*move_group1, robotPose[1][7]);
            break;
        case 17:
            robotMoveCartesian(*move_group1, 0, prepare_some_distance * 0.707, -prepare_some_distance * 0.707);
            setAndMove(*move_group1, robotPose[1][0]);
            setAndMove(*move_group0, photographPose[9]);
            break;
        }
    }
    else if (recordPointData.model == 1)
    {
        switch (recordPointData.stepNum)
        {
        case 1:
            closeGripper(*move_group1);
            robotMoveCartesian(*move_group1, 0, 0, prepare_some_distance);
            setAndMove(*move_group1, photographPose[1]);
            setAndMove(*move_group0, photographPose[2]);
            break;
        case 2:
            setAndMove(*move_group1, photographPose[3]);
            setAndMove(*move_group0, photographPose[4]);
            break;
        case 3:
            setAndMove(*move_group0, robotPose[0][0]);
            setAndMove(*move_group1, robotPose[1][3]);
            robotMoveCartesian(*move_group1, 0, -prepare_some_distance, 0);
            analyseData(3, 0);
            swop(getMoveGroup(Adata.captureRobot), getMoveGroup(Adata.otherRobot), robotPose[Adata.captureRobot][Adata.capturePoint]);
            setAndMove(*move_group0, photographPose[5]);
            setAndMove(*move_group1, photographPose[6]);
            break;
        case 4:
            setAndMove(*move_group0, photographPose[7]);
            setAndMove(*move_group1, photographPose[8]);
            break;
        }
    }
    return 0;
}

int RubikCubeSolve::Cartesian()
{
    int f = 0x0001;
    f <<= recordPointData.stepNum;
    if (overStepflag >> recordPointData.stepNum)
        return 0;
    else
        overStepflag |= f;
    if (recordPointData.stepNum == 0)
        robotMoveCartesianUnitClient(*move_group1, 0, 0, -prepare_some_distance);
    if (recordPointData.model == 0)
    {
        if ((recordPointData.stepNum >= 4 && recordPointData.stepNum <= 7) || recordPointData.stepNum == 10)
            robotMoveCartesianUnitClient(*move_group0, 0, prepare_some_distance, 0);
        else if (recordPointData.stepNum >= 11 && recordPointData.stepNum <= 14)
            robotMoveCartesianUnitClient(*move_group1, 0, -prepare_some_distance, 0);
        else if (recordPointData.stepNum == 9)
            robotMoveCartesianUnitClient(*move_group0, 0, prepare_some_distance * 0.707, prepare_some_distance * 0.707);
        else if (recordPointData.stepNum == 16)
            robotMoveCartesianUnitClient(*move_group1, 0, -prepare_some_distance * 0.707, prepare_some_distance * 0.707);
    }
    else if (recordPointData.stepNum == 17)
    {
        robotMoveCartesianUnitClient(*move_group0, 0, 0, -prepare_some_distance);
    }
    return 0;
}

// int RubikCubeSolve::updataPointData()
// {
//     if (recordPointData.stepNum == 0)
//     {
//         recordPose(1, recordPointData.poseName[recordPointData.stepNum], false);
//     }
//     else if (recordPointData.model == 0)
//     {
//         std::vector<int> robot0 = {2, 4, 5, 6, 7, 9, 10, 15, 17};
//         std::vector<int> robot1 = {0, 1, 3, 8, 11, 12, 13, 14, 16};
//         for (int i : robot0)
//             if (recordPointData.stepNum == i)
//             {
//                 recordPose(0, recordPointData.poseName[recordPointData.stepNum], false);
//                 return 0;
//             }
//         recordPose(1, recordPointData.poseName[recordPointData.stepNum], false);
//         if (recordPointData.stepNum == 3)
//         {
//             geometry_msgs::PoseStamped newPoseStamped;
//             newPoseStamped = move_group1->getCurrentPose();
//             newPoseStamped.pose.position.y += prepare_some_distance;
//             robotPose[1][3] = newPoseStamped;
//         }
//     }
//     else if (recordPointData.model == 1)
//     {
//         int captureCubeRobot;
//         int otherRobot;
//         if (recordPointData.stepNum == 1 || recordPointData.stepNum == 2)
//         {
//             captureCubeRobot = 1;
//             otherRobot = 0;
//         }
//         else if (recordPointData.stepNum == 3 || recordPointData.stepNum == 4)
//         {
//             captureCubeRobot = 0;
//             otherRobot = 1;
//         }
//         recordPose(captureCubeRobot, recordPointData.shootPhotoPoseName[recordPointData.stepNum * 2 - 2], false);
//         recordPose(otherRobot, recordPointData.shootPhotoPoseName[recordPointData.stepNum * 2 - 1], false);
//     }
//     return 0;
// }

void RubikCubeSolve::robotMoveCartesian(moveit::planning_interface::MoveGroupInterface &group, double x, double y, double z)
{
    ROS_INFO_STREAM("robotMoveCartesian: " << group.getName());
    ros::Duration(1).sleep();
    setStartState(group);
    geometry_msgs::PoseStamped temp_pose = group.getCurrentPose();

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x += x;
    temp_pose.pose.position.y += y;
    temp_pose.pose.position.z += z;
    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    while (group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1.0 && !isStop && ros::ok())
        ;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (!isStop)
        group.execute(plan);
    ROS_INFO_STREAM("robotMoveCartesian over");
}

bool RubikCubeSolve::setAndMoveMulti(moveit::planning_interface::MoveGroupInterface &group,
                                     std::vector<geometry_msgs::PoseStamped> &poses, int type,
                                     trajectory_msgs::JointTrajectory &tra,
                                     bool only_plan = true)
{
    ROS_INFO_STREAM("setAndMoveMulti: " << group.getName() << " "
                                        << "only_plan: " << only_plan);
    setStartState(group);
    ROS_INFO_STREAM("type: " << type);
    bool flag;
    std::vector<moveit_msgs::RobotTrajectory> trajectory;
    int cnt = poses.size();
    trajectory.resize(cnt);
    // 获取现在状态
    robot_state::RobotState r(*group.getCurrentState());
    const robot_state::JointModelGroup *jointGroup = r.getJointModelGroup(group.getName());
    std::vector<double> joints = group.getCurrentJointValues();
    r.setJointGroupPositions(jointGroup, joints);
    // 选择轨迹组合的类型
    // 0: 曲线 + 直线
    // 1: 直线 + 曲线
    // 2: N个曲线
    if (type == 0)
    {
        // 获取轨迹
        setJointAllConstraints(group);
        if (!RobotTrajectory(group, poses[0], trajectory[0], r))
        {
            ROS_INFO_STREAM("get Trajectory failed" << 0);
            return false;
        }
        ROS_INFO_STREAM("get Trajectory " << 0);
        clearConstraints(group);

        int end = trajectory[0].joint_trajectory.points.size() - 1;
        r.setJointGroupPositions(jointGroup, trajectory[0].joint_trajectory.points[end].positions);

        if (!RobotTrajectoryLine(group, poses[1], trajectory[1], r))
        {
            ROS_INFO_STREAM("get Trajectory failed" << 1);
            return false;
        }
        ROS_INFO_STREAM("get Trajectory " << 1);
    }
    else if (type == 1)
    {
        if (!RobotTrajectoryLine(group, poses[0], trajectory[0], r))
        {
            ROS_INFO_STREAM("get Trajectory failed" << 0);
            return false;
        }
        ROS_INFO_STREAM("get Trajectory " << 0);
        int end = trajectory[0].joint_trajectory.points.size() - 1;
        r.setJointGroupPositions(jointGroup, trajectory[0].joint_trajectory.points[end].positions);
        // 获取轨迹
        setJointAllConstraints(group);
        if (!RobotTrajectory(group, poses[1], trajectory[1], r))
        {
            ROS_INFO_STREAM("get Trajectory failed" << 1);
            return false;
        }
        ROS_INFO_STREAM("get Trajectory " << 1);
        clearConstraints(group);
    }
    else if (type == 2)
    {
        setJointAllConstraints(group);
        for (int i = 0; i < cnt; ++i)
        {
            if (!RobotTrajectory(group, poses[i], trajectory[i], r))
            {
                ROS_INFO_STREAM("get Trajectory failed" << i);
                clearConstraints(group);
                return false;
            }
            ROS_INFO_STREAM("get Trajectory " << i);
            int end = trajectory[i].joint_trajectory.points.size() - 1;
            r.setJointGroupPositions(jointGroup, trajectory[i].joint_trajectory.points[end].positions);
        }
        clearConstraints(group);
    }

    // 拼接轨迹
    moveit_msgs::RobotTrajectory targetTrajectory;
    targetTrajectory.joint_trajectory.joint_names = trajectory[0].joint_trajectory.joint_names;
    targetTrajectory.joint_trajectory.points = trajectory[0].joint_trajectory.points;
    for (int j = 1; j < cnt; j++)
    {
        for (int k = 1; k < trajectory[j].joint_trajectory.points.size(); ++k)
        {
            targetTrajectory.joint_trajectory.points.push_back(trajectory[j].joint_trajectory.points[k]);
        }
    }
    //
    moveit::planning_interface::MoveGroupInterface::Plan multi_plan;
    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());

    rt.setRobotTrajectoryMsg(*(group.getCurrentState()), targetTrajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.5, 0.5);

    rt.getRobotTrajectoryMsg(multi_plan.trajectory_);

    // trajectory_msgs::JointTrajectory tra;
    if (only_plan)
        tra = multi_plan.trajectory_.joint_trajectory;
    else
    {
        // 执行
        std::vector<trajectory_msgs::JointTrajectory> tra;
        tra.resize(2);
        setStartState(group);
        // if (!group.execute(multi_plan))
        // {
        //     ROS_ERROR("Failed to execute plan");
        //     system("rosrun rubik_cube_solve set_robot_enable_false.sh");
        //     return false;
        // }
        // ROS_ERROR("SUCCEED to execute plan");
        if (group.getName() == "arm0")
        {
            tra[0] = multi_plan.trajectory_.joint_trajectory;
        }
        else
        {
            tra[1] = multi_plan.trajectory_.joint_trajectory;
        }
        seedTrajectory(tra[0], tra[1]);
    }
    ROS_INFO_STREAM("setAndMoveMulti over");
    return true;
}

bool RubikCubeSolve::RobotTrajectory(moveit::planning_interface::MoveGroupInterface &group,
                                     geometry_msgs::PoseStamped &TargetPose,
                                     moveit_msgs::RobotTrajectory &trajectory,
                                     robot_state::RobotState &r)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool flag = false;
    // 规划
    group.setStartState(r);
    group.setPoseTarget(TargetPose);
    int cnt = 0;
    while (!flag && ros::ok() && cnt < 5)
    {
        flag = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        cnt++;
    }
    if (flag)
    {
        trajectory.joint_trajectory.joint_names = my_plan.trajectory_.joint_trajectory.joint_names;
        trajectory.joint_trajectory.points = my_plan.trajectory_.joint_trajectory.points;
    }
    return flag;
}

bool RubikCubeSolve::RobotTrajectoryLine(moveit::planning_interface::MoveGroupInterface &group,
                                         geometry_msgs::PoseStamped &TargetPose,
                                         moveit_msgs::RobotTrajectory &trajectory,
                                         robot_state::RobotState &r)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool flag;
    // 规划
    group.setStartState(r);
    group.setPoseTarget(TargetPose);
    std::vector<geometry_msgs::Pose> pose;
    pose.push_back(TargetPose.pose);
    int cnt = 0;
    double percent = 0;
    while (percent < 0.8 && ros::ok() && ++cnt < 10)
    {
        percent = group.computeCartesianPath(pose, 0.01, 0, trajectory);
    }
    if (cnt == 10 && percent < 0.8)
        return false;
    return true;
}

bool RubikCubeSolve::seedTrajectory(trajectory_msgs::JointTrajectory &robot0Trajectory, trajectory_msgs::JointTrajectory &robot1Trajectory)
{
    ros::Duration(1.0).sleep();
    // hirop_msgs::dualRbtraject srv;
    // bool flag;
    // ROS_ERROR_STREAM("Trajectory0: " << robot0Trajectory.points.size());
    // ROS_ERROR_STREAM("Trajectory1: " << robot1Trajectory.points.size());

    // srv.request.robotMotionTraject_list.resize(2);
    // srv.request.robotMotionTraject_list[0].moveGroup_name = move_group0.getName();
    // srv.request.robotMotionTraject_list[0].robot_jointTra = robot0Trajectory;
    // srv.request.robotMotionTraject_list[1].moveGroup_name = move_group1.getName();
    // srv.request.robotMotionTraject_list[1].robot_jointTra = robot1Trajectory;
    // if(dualRobottrajectory.call(srv))
    // {
    //     flag = srv.response.is_success;
    // }
    // else
    // {
    //     flag = false;
    // }
    // ROS_INFO_STREAM("seedTrajectory over");
    // return flag;
    ROS_INFO("---->>>>----");
    moveit_msgs::RobotTrajectory j0;
    moveit_msgs::RobotTrajectory j1;
    j0.joint_trajectory = robot0Trajectory;
    j1.joint_trajectory = robot1Trajectory;
    moveit::planning_interface::MoveGroupInterface::Plan plan0;
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    plan0.trajectory_ = j0;
    plan1.trajectory_ = j1;
    move_group0->execute(plan0);
    move_group1->execute(plan1);
    return true;
}
