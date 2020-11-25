#include "vision_detection.h"

VisionDetection::VisionDetection(ros::NodeHandle *n)
{
    nh = n;
    nh->param("arm", arm, string("arm"));
    nh->param("tip", tip, string("link6"));
    nh->param("targetFrame", targetFrame, string("base_link"));
//    nh->param("targetFrame", targetFrame, string("world"));
    objectArraySub = nh->subscribe("/object_array", 10, &VisionDetection::objectArrayCB, this);
    anglePub = nh->advertise<hirop_msgs::joints_angle>("/joint_angle", 1);
    detectionClient = nh->serviceClient<hirop_msgs::detection>("detection");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    robot_state = new robot_state::RobotState(kinematic_model);
    joint_model_group = robot_state->getJointModelGroup(arm);
//    nh->setParam("/compensate_x", 0.2);
//    nh->setParam("/compensate_y", 0.0);
//    nh->setParam("/compensate_z", 0.28);
//    nh->setParam("/compensate_x", 0.05);
//    nh->setParam("/compensate_y", 0.055);
//    nh->setParam("/compensate_z", 0.275);
//    nh->setParam("/compensate_x", 0.05);
    nh->setParam("/compensate_x", 0.0);
    nh->setParam("/compensate_y", 0.0);
    nh->setParam("/compensate_z", 0.28);
    nh->setParam("/compensate_theta", -22.0);
}

VisionDetection::~VisionDetection()
{
}

int VisionDetection::detection(string object, string detector)
{
    hirop_msgs::detection srv;
    srv.request.detectorType = 0;
    srv.request.objectName = object;
    srv.request.detectorName = detector;
    vector<double>().swap(resultAngle);
    if (detectionClient.call(srv))
    {
        return srv.response.result;
    }
    return -1;
}
int VisionDetection::compensatin_x(double x)
{
    nh->setParam("/compensate_x", x);
    return 0;
}
int VisionDetection::compensatin_y(double y)
{
    nh->setParam("/compensate_y", y);
    return 0;
}
int VisionDetection::compensatin_z(double z)
{
    nh->setParam("/compensate_z", z);
    return 0;
}
int VisionDetection::getJointAngleResult(vector<double> &joints)
{
    cout<<"进入getJointAngleResult"<<endl;
    int cnt = 0;
    if(resultAngle.empty()){
        cout<<"resultAngle是empty"<<endl;
    } else{
        cout<<"resultAngle不是empty "<<endl;
    }
    ros::Duration(1).sleep();
    while (resultAngle.empty())
    {
        ros::Duration(0.5).sleep();
        cout<<"循环等待resultAngle"<<endl;
        if(ArrayFlag == -1 || cnt > 10) {
            break;
        }
        cnt ++;
        cout<<"cnt   "<<cnt<<endl;
    }
    cout << "resultAngler.size: " << resultAngle.size() << endl;
    if(!resultAngle.empty())
    {
        joints = resultAngle;
        return 0;
    }
    return -1;
}

void VisionDetection::objectArrayCB(const hirop_msgs::ObjectArrayConstPtr &msg)
{
    ROS_INFO("into CB----------------");
    ArrayFlag = 0;
    geometry_msgs::PoseStamped soucePose = msg->objects[0].pose;
//    soucePose.pose.position.z=0.324577;
    soucePose.pose.position.z=0.339577;
    geometry_msgs::PoseStamped targetPose;
    if (transformFrame(soucePose, targetPose, targetFrame) == 0)
    {
//        targetPose.pose.orientation.x = 0;
//        targetPose.pose.orientation.y = 0;
//        targetPose.pose.orientation.z = 0;
//        targetPose.pose.orientation.w = 1;
        vector<double> joints;
        if (IK(targetPose, joints, tip) == 0)
        {
            hirop_msgs::joints_angle Jmsg;
            resultAngle = rad2angle(joints);
            //第六轴补偿一定角度
            cout<<"角度补偿"<<endl;
            double thta=0;
            nh->getParam("/compensate_theta", thta);
            resultAngle[5]+=180+thta;
            cout<<"补偿角度为："<<resultAngle[5]<<endl;
            for (int i = 0; i <2; ++i) {
                if(resultAngle[5]>=180)
                {
                    resultAngle[5]-=360;
                }
            }
            //把1轴转成正数
            if(resultAngle[0]<=0){
                resultAngle[0]+=360;
            }


            Jmsg.joints_angle.data = resultAngle;
            anglePub.publish(Jmsg);
        }
    }
    ArrayFlag = -1;
}

int VisionDetection::transformFrame(const geometry_msgs::PoseStamped &p, geometry_msgs::PoseStamped &target, const string &frame_id)
{
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped source_pose;

    tf::TransformListener tf_listener;

    source_pose = p;
    cout << " camera_color_optical_frame: " << frame_id << endl;
    for (int i = 0; i < 5; ++i)
    {
        try
        {
            tf_listener.transformPose(frame_id, source_pose, target_pose);
            break;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("transfrom exception : %s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
    }
    target = target_pose;
//    nh->param("compensate_x", wX, double(0.0));
//    nh->param("compensate_y", wY, double(0.0));
//    nh->param("compensate_z", wZ, double(0.2));
    nh->getParam("/compensate_x", wX);
    nh->getParam("/compensate_y", wY);
    nh->getParam("/compensate_z", wZ);
    target.pose.position.x += wX;
    target.pose.position.y += wY;
    target.pose.position.z += wZ;
    if (target.header.frame_id == frame_id)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

vector<double> VisionDetection::rad2angle(vector<double> rad)
{
    vector<double> angle;
    for (auto i : rad)
    {
        angle.push_back(i / M_PI * 180);
    }
    return angle;
}

int VisionDetection::IK(geometry_msgs::PoseStamped &pose, vector<double> &joints, string tip_)
{
    int flag = -1;
    size_t attempts = 10;
    double timeout = 0.5;
    int cnt = 0;
    while (ros::ok() && cnt < 5)
    {
        cout<<"开始逆解"<<endl;
        if (robot_state->setFromIK(joint_model_group, pose.pose, tip_, attempts, timeout))
        {
            ROS_INFO_STREAM("IK succeed");
            flag = 0;
            robot_state->copyJointGroupPositions(joint_model_group, joints);
            break;
        }
        cnt++;
        ROS_INFO_STREAM("IK cnt: " << cnt);
    }
    return flag;
}