#include "navigation.h"

Navigation::Navigation(ros::NodeHandle* n)
{
    nh = n;
    cancelPub = nh->advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
    currentMap = "map1";
}

int Navigation::moveTo(string flag)
{
    // Timer timer("navigation");

    PoseData* data;

    data = getMapFlagPose(flag);

    /**
     * @brief 获取移动底盘的实例
     */
    MobileRobot *robot = MobileRobot::getInstance();

    /**
     * 创建一个导航运动
     */
    AutoRunMotion *motion = new AutoRunMotion();

    /**
     *  设置目标点
     */
    motion->setTargetPose(data->pose);

    /**
     *  执行运动
     */
    int result;
    result = robot->runMotion(motion,true);

    delete data;
    delete motion;
    return result;
}

int Navigation::clearCostMap()
{
    tf::TransformListener tf(ros::Duration(10));

    costmap_2d::Costmap2DROS global_costmap("global_costmap", tf);
    costmap_2d::Costmap2DROS local_costmap("local_costmap", tf);

    clear_costmap_recovery::ClearCostmapRecovery ccr;
    ccr.initialize("my_clear_costmap_recovery", &tf, &global_costmap, &local_costmap);

    ccr.runBehavior();
}

int Navigation::mappingIteration(int iter)
{
    std_srvs::Empty srv;
    for(int i=0; i < iter; i++)
    {
//        if(!request_nomotion_update_client.call(srv))
//            return -1;
        system("rosservice call /request_nomotion_update \"{}\"");
    }
    return 0;
}

int Navigation::stop()
{
    actionlib_msgs::GoalID msg;
    cancelPub.publish(msg);
    stopFlag = true;
    return 0;
}
PoseData * Navigation::getMapFlagPose(std::string flag)
{
    FileDataWriter write;

    DataUri uri(flag);
    uri.addFlag("MAP");
    uri.addFlag(currentMap);
    uri.addFlag("flags");

    PoseData *data = (PoseData *)write.loadData(uri);

    return data;
}
