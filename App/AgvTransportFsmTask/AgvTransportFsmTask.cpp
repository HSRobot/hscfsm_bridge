#include "AgvTransportFsmTask.h"

using namespace HsFsm;
typedef std::pair<string, int> elment;

AgvTransportFsmTask::AgvTransportFsmTask() {
//    nav=new Navigation(getRosHandler());
//    visionDetection =new VisionDetection(getRosHandler());
//    hsc3RobotMove=new Hsc3RobotMove();
//    outSwapModule =new OutSwapModule();
}

AgvTransportFsmTask::AgvTransportFsmTask(const string &taskName)
{
    this->taskName = taskName;

}

void AgvTransportFsmTask::init()
{

    // 自动转为 init 状态
    // 用户自定义添加内容
    cout<<" 自动转为 init 状态"<<endl;
    cout<<" AgvTransportFsmTask"<<endl;
    goalResult_sub=getRosHandler()->subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result",1000,&AgvTransportFsmTask::callback_agvResult_subscriber,this);
    joyKey_sub=getRosHandler()->subscribe<sensor_msgs::Joy>("/joy_teleop/joy",1,&AgvTransportFsmTask::callback_joyKey_sub_subscriber,this);
    robotEnable = getRosHandler()->serviceClient<hsr_rosi_device::SetEnableSrv>("/set_robot_enable");
    serviceClient_serialOpen=getRosHandler()->serviceClient<hsr_gripper_driver::serial_open_srv>("/serial_open");
    serviceClient_gripperOpen=getRosHandler()->serviceClient<hsr_gripper_driver::open_srv>("/gripper_open");
    serviceClient_findbase=getRosHandler()->serviceClient<hsr_gripper_driver::open_srv>("/gripper_find_base_size");
    serviceClient_gripperClose=getRosHandler()->serviceClient<hsr_gripper_driver::close_srv>("/gripper_close");
    vel_pub = getRosHandler()->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    srv_enable.request.enable= false;
    enable_flag= false;
    //串口连接
    srv_serial_open.request.baudrate=115200;
    srv_serial_open.request.serialNo="/dev/ttyUSB0";

    srv_closeGripper.request.force=1000;
    srv_closeGripper.request.speed=800;

    srv_openGripper.request.speed=500;
    srv_findbase.request.speed=0;
    pick_poseList.push_back("pick1");
    pick_poseList.push_back("pick2");
//    pick_poseList.push_back("pick3");

    place_poseList.push_back("place1");
    place_poseList.push_back("place2");
//    place_poseList.push_back("place3");

}

void AgvTransportFsmTask::quit()
{
    cout<<"没用的quit"<<endl;
    //会自动转状态为 quit
    // 用户自定义添加内容
}

/**
 * @brief shakeHandTask::registerTaskList 用户重点关心
 * @return
 */
//
bool AgvTransportFsmTask::registerTaskList()
{
    auto func_init_initing = std::bind(&AgvTransportFsmTask::Init_initing, this, placeholders::_1);
    auto func_init_quiting = std::bind(&AgvTransportFsmTask::Init_quiting, this, placeholders::_1);
    auto func_init_toSelfCheck = std::bind(&AgvTransportFsmTask::Init_toSelfCheck, this, placeholders::_1);

    auto func_SelfCheck_initing = std::bind(&AgvTransportFsmTask::SelfCheck_initing, this, placeholders::_1);
    auto func_SelfCheck_quiting = std::bind(&AgvTransportFsmTask::SelfCheck_quiting, this, placeholders::_1);
    auto func_SelfCheck_toExit = std::bind(&AgvTransportFsmTask::SelfCheck_toExit, this, placeholders::_1);
    auto func_SelfCheck_toErr = std::bind(&AgvTransportFsmTask::SelfCheck_toErr, this, placeholders::_1);

    auto func_AgvMoveToPick_initing = std::bind(&AgvTransportFsmTask::AgvMoveToPick_initing, this, placeholders::_1);
    auto func_AgvMoveToPick_quiting = std::bind(&AgvTransportFsmTask::AgvMoveToPick_quiting, this, placeholders::_1);
    auto func_AgvMoveToPick_toExit = std::bind(&AgvTransportFsmTask::AgvMoveToPick_toExit, this, placeholders::_1);
    auto func_AgvMoveToPick_toErr = std::bind(&AgvTransportFsmTask::AgvMoveToPick_toErr, this, placeholders::_1);

    auto func_WaitForBasket_initing = std::bind(&AgvTransportFsmTask::WaitForBasket_initing, this, placeholders::_1);
    auto func_WaitForBasket_quiting = std::bind(&AgvTransportFsmTask::WaitForBasket_quiting, this, placeholders::_1);
    auto func_WaitForBasket_toExit = std::bind(&AgvTransportFsmTask::WaitForBasket_toExit, this, placeholders::_1);
    auto func_WaitForBasket_toErr = std::bind(&AgvTransportFsmTask::WaitForBasket_toErr, this, placeholders::_1);

    auto func_DetectObjPose_initing = std::bind(&AgvTransportFsmTask::DetectObjPose_initing, this, placeholders::_1);
    auto func_DetectObjPose_quiting = std::bind(&AgvTransportFsmTask::DetectObjPose_quiting, this, placeholders::_1);
    auto func_DetectObjPose_toExit = std::bind(&AgvTransportFsmTask::DetectObjPose_toExit, this, placeholders::_1);
    auto func_DetectObjPose_toErr = std::bind(&AgvTransportFsmTask::DetectObjPose_toErr, this, placeholders::_1);

    auto func_RobPickBasket_initing = std::bind(&AgvTransportFsmTask::RobPickBasket_initing, this, placeholders::_1);
    auto func_RobPickBasket_quiting = std::bind(&AgvTransportFsmTask::RobPickBasket_quiting, this, placeholders::_1);
    auto func_RobPickBasket_toExit = std::bind(&AgvTransportFsmTask::RobPickBasket_toExit, this, placeholders::_1);
    auto func_RobPickBasket_toErr = std::bind(&AgvTransportFsmTask::RobPickBasket_toErr, this, placeholders::_1);

    auto func_AgvMoveToPlace_initing = std::bind(&AgvTransportFsmTask::AgvMoveToPlace_initing, this, placeholders::_1);
    auto func_AgvMoveToPlace_quiting = std::bind(&AgvTransportFsmTask::AgvMoveToPlace_quiting, this, placeholders::_1);
    auto func_AgvMoveToPlace_toExit = std::bind(&AgvTransportFsmTask::AgvMoveToPlace_toExit, this, placeholders::_1);
    auto func_AgvMoveToPlace_toErr = std::bind(&AgvTransportFsmTask::AgvMoveToPlace_toErr, this, placeholders::_1);

    auto func_WaitForPlacePoseEmpty_initing = std::bind(&AgvTransportFsmTask::WaitForPlacePoseEmpty_initing, this, placeholders::_1);
    auto func_WaitForPlacePoseEmpty_quiting = std::bind(&AgvTransportFsmTask::WaitForPlacePoseEmpty_quiting, this, placeholders::_1);
    auto func_WaitForPlacePoseEmpty_toExit = std::bind(&AgvTransportFsmTask::WaitForPlacePoseEmpty_toExit, this, placeholders::_1);
    auto func_WaitForPlacePoseEmpty_toErr = std::bind(&AgvTransportFsmTask::WaitForPlacePoseEmpty_toErr, this, placeholders::_1);

    auto func_RobPlaceBasket_initing = std::bind(&AgvTransportFsmTask::RobPlaceBasket_initing, this, placeholders::_1);
    auto func_RobPlaceBasket_quiting = std::bind(&AgvTransportFsmTask::RobPlaceBasket_quiting, this, placeholders::_1);
    auto func_RobPlaceBasket_toExit = std::bind(&AgvTransportFsmTask::RobPlaceBasket_toExit, this, placeholders::_1);
    auto func_RobPlaceBasket_toErr = std::bind(&AgvTransportFsmTask::RobPlaceBasket_toErr, this, placeholders::_1);

    auto func_Exit_initing = std::bind(&AgvTransportFsmTask::Exit_initing, this, placeholders::_1);
    auto func_Exit_quiting = std::bind(&AgvTransportFsmTask::Exit_quiting, this, placeholders::_1);

    auto func_Err_initing = std::bind(&AgvTransportFsmTask::Err_initing, this, placeholders::_1);
    auto func_Err_quiting = std::bind(&AgvTransportFsmTask::Err_quiting, this, placeholders::_1);
    auto func_Err_toInit = std::bind(&AgvTransportFsmTask::Err_toInit, this, placeholders::_1);

    auto func_loop_initing = std::bind(&AgvTransportFsmTask::loop_initing, this, placeholders::_1);

    try{
        registerTask("init","initing", func_init_initing);
        registerTask("init","quiting", func_init_quiting);
        registerTask("init","toSelfCheck", func_init_toSelfCheck);
        registerTask("loop","initing", func_loop_initing);
        registerTask("test","initing", [&](callParm  &parm)
        {
            return;
            //串口连接
            serviceClient_serialOpen.call(srv_serial_open);
            sleep(2);
            //打开夹爪
            serviceClient_gripperClose.call(srv_closeGripper);
            cout<<"关闭夹爪"<<endl;
            sleep(2);
            //关闭夹爪
            serviceClient_gripperOpen.call(srv_openGripper);
            cout<<"打开夹爪"<<endl;
            sleep(2);
            int a;
            for (int j = 0; j <100; ++j) {
                cout<<"请输入数据"<<endl;
                cin>>a;
                if(a==1){
                    cout<<"关闭夹爪！"<<endl;
                    serviceClient_findbase.call(srv_findbase);
                    sleep(1);
                    serviceClient_gripperClose.call(srv_closeGripper);
                    sleep(2);
                } else{
                    serviceClient_gripperOpen.call(srv_openGripper);
                    cout<<"打开夹爪"<<endl;
                    sleep(2);
                }
            }
            return;
//            avg_turn180degree();
//            agv_gostright();
//            return;

//            for (int j = 1; j <=10; ++j)
//            {
//                if(smartToNav_forplace()!=0){
//                    return;
//                }
//                //掉头
//                cout<<"掉头"<<endl;
//                avg_turn180degree();
//                sleep(1);
//                if(smartToNav_forpick()!=0){
//                    return;
//                }
//                //掉头
//                cout<<"掉头"<<endl;
//                sleep(1);
//                avg_turn180degree();
//
//                cout<<"第"<<j<<"次进行完成 "<<endl;
//            }
//            return;

             cout<<"设备连接"<<endl;
            Hsc3apiInstance::getInstance()->connect("192.168.99.3",23234);
            //2.目标检测
            for (int i = 0; i < 4; ++i)
            {
                if(visionDetection->detection("ar","Ar")==0)
                {
        //            setTaskState("Err");
                    break;
        //            return;
                }else
                {
                    sleep(1);
                }
            }
                    vector<double > targetPose(6);
                    cout<<"获取结果"<<endl;
                    if(visionDetection->getJointAngleResult(targetPose)!=0){
                        cout<<"获取关节角错误 "<<endl;
                        setTaskState("Err");
                        return;
                    }
                    cout<<"识别到目标点J[0]:"<<targetPose[0]<<endl;
                    cout<<"识别到目标点J[1]:"<<targetPose[1]<<endl;
                    cout<<"识别到目标点J[2]:"<<targetPose[2]<<endl;
                    cout<<"识别到目标点J[3]:"<<targetPose[3]<<endl;
                    cout<<"识别到目标点J[4]:"<<targetPose[4]<<endl;
                    cout<<"识别到目标点J[5]:"<<targetPose[5]<<endl;
                    hsc3RobotMove->setJR_jointpose(0,4,targetPose);
        });

        registerTask("SelfCheck","initing", func_SelfCheck_initing);//to AgvMoveToPick
        registerTask("SelfCheck","quiting", func_SelfCheck_quiting);
        registerTask("SelfCheck","toExit", func_SelfCheck_toExit);
        registerTask("SelfCheck","toErr", func_SelfCheck_toErr);

        registerTask("AgvMoveToPick","initing", func_AgvMoveToPick_initing);//to WaitForBasket
        registerTask("AgvMoveToPick","quiting", func_AgvMoveToPick_quiting);
        registerTask("AgvMoveToPick","toExit", func_AgvMoveToPick_toExit);
        registerTask("AgvMoveToPick","toErr", func_AgvMoveToPick_toErr);

        registerTask("WaitForBasket","initing", func_WaitForBasket_initing);//to DetectObjPose
        registerTask("WaitForBasket","quiting", func_WaitForBasket_quiting);
        registerTask("WaitForBasket","toExit", func_WaitForBasket_toExit);
        registerTask("WaitForBasket","toErr", func_WaitForBasket_toErr);

        registerTask("DetectObjPose","initing", func_DetectObjPose_initing);//to RobPickBasket
        registerTask("DetectObjPose","quiting", func_DetectObjPose_quiting);
        registerTask("DetectObjPose","toExit", func_DetectObjPose_toExit);
        registerTask("DetectObjPose","toErr", func_DetectObjPose_toErr);

        registerTask("RobPickBasket","initing", func_RobPickBasket_initing); //to AgvMoveToPlace
        registerTask("RobPickBasket","quiting", func_RobPickBasket_quiting);
        registerTask("RobPickBasket","toExit", func_RobPickBasket_toExit);
        registerTask("RobPickBasket","toErr", func_RobPickBasket_toErr);

        registerTask("AgvMoveToPlace","initing", func_AgvMoveToPlace_initing); //to WaitForPlacePoseEmpty
        registerTask("AgvMoveToPlace","quiting", func_AgvMoveToPlace_quiting);
        registerTask("AgvMoveToPlace","toExit", func_AgvMoveToPlace_toExit);
        registerTask("AgvMoveToPlace","toErr", func_AgvMoveToPlace_toErr);

        registerTask("WaitForPlacePoseEmpty","initing", func_WaitForPlacePoseEmpty_initing); //to RobPlaceBasket
        registerTask("WaitForPlacePoseEmpty","quiting", func_WaitForPlacePoseEmpty_quiting);
        registerTask("WaitForPlacePoseEmpty","toExit", func_WaitForPlacePoseEmpty_toExit);
        registerTask("WaitForPlacePoseEmpty","toErr", func_WaitForPlacePoseEmpty_toErr);

        registerTask("RobPlaceBasket","initing", func_RobPlaceBasket_initing); //to AgvMoveToPick
        registerTask("RobPlaceBasket","quiting", func_RobPlaceBasket_quiting);
        registerTask("RobPlaceBasket","toExit", func_RobPlaceBasket_toExit);
        registerTask("RobPlaceBasket","toErr", func_RobPlaceBasket_toErr);

        registerTask("Exit","initing", func_Exit_initing);//to init
        registerTask("Exit","quiting", func_Exit_quiting);

        registerTask("Err","initing", func_Err_initing);
        registerTask("Err","quiting", func_Err_quiting);
        registerTask("Err","toInit", func_Err_toInit);


    }catch(std::exception &e)
    {
        std::cout  << e.what()<<std::endl;
        assert(-1);
        return false;
    }

    return true;
}



/******************************状态行为函数**************************************************/
//init状态
void AgvTransportFsmTask::Init_initing(const std::vector<std::string> &args) {
    system("rosservice call /clear_robot_fault \"{}\"");
    srv_enable.request.enable= true;
    robotEnable.call(srv_enable);
    setTaskState("test");
    return;
    isEixt= false;
    isErr= false;
    //串口连接
    serviceClient_serialOpen.call(srv_serial_open);
    sleep(2);
    //打开夹爪
    serviceClient_gripperClose.call(srv_closeGripper);
    cout<<"关闭夹爪"<<endl;
    sleep(2);
    //关闭夹爪
    serviceClient_gripperOpen.call(srv_openGripper);
    cout<<"打开夹爪"<<endl;
    sleep(2);

//    return;

    hsc3RobotMove->init();
//    setTaskState("test");
    setTaskState("loop");
    cout<<"init over"<<endl;

}

void AgvTransportFsmTask::loop_initing(const std::vector<std::string> &args){
    for (int j = 1; j <=30; ++j) {
        if(smartToNav_forpick()!=0){
            cout<<"智能导航pick点失败"<<endl;
            return;
        }
        //掉头
        cout<<"掉头"<<endl;
        avg_turn180degree();
        sleep(1);
//    //等待抓取点药品篮子到位
    while (!outSwapModule->pickPoseHasBaskect())
    {
        if(isEixt)
        {
            setTaskState("Exit");
            return;
        }
        sleep(1);
        cout<<"等待篮子到位信号R[31]"<<endl;
    }

    //1.去到检测姿势点
    bool is_dectionSucess= false;
    int num=0;
    int32_t index_pose[]{11,16,17,18};
    while (num<4){
        if(hsc3RobotMove->RobGoToDetectPose(index_pose[num])!=0)
        {
            cout<<"去到拍照点故障"<<endl;
            return;
        }
        num++;
        sleep(1);
        //2.目标检测
        for (int i = 0; i < 4; ++i) {
            if(visionDetection->detection("ar","Ar")==0)
            {
                cout<<"识别故障"<<endl;
                break;
            }else{
                sleep(1);
            }
        }
        vector<double > targetPose(6);
        cout<<"DetectObjPose_initing 获取结果"<<endl;
        if(visionDetection->getJointAngleResult(targetPose)!=0){
            cout<<"识别不到二维码"<<endl;
            continue;
        }
        cout<<"识别到目标点J[0]:"<<targetPose[0]<<endl;
        cout<<"识别到目标点J[1]:"<<targetPose[1]<<endl;
        cout<<"识别到目标点J[2]:"<<targetPose[2]<<endl;
        cout<<"识别到目标点J[3]:"<<targetPose[3]<<endl;
        cout<<"识别到目标点J[4]:"<<targetPose[4]<<endl;
        cout<<"识别到目标点J[5]:"<<targetPose[5]<<endl;
        hsc3RobotMove->setJR_jointpose(0,4,targetPose);
        is_dectionSucess=true;
        break;
    }

    if(!is_dectionSucess){
        cout<<"识别故障"<<endl;
        return;
    }

    if(isErr){
        setTaskState("Err");
        return;
    }

    //线程一直监听
    double value1=0;
    double value2=0;
    bool stop_flag= false;
    std::thread t1([&]{
        cout<<"thread"<<endl;
        int sleep_once_count=0;
        while (ros::ok()&&(!stop_flag))
        {
            Hsc3apiInstance::getInstance()->getR(40,value1); //等待通知夹爪关闭
            if(value1==1.0)
            {
                cout<<"关闭夹爪！"<<endl;
                serviceClient_findbase.call(srv_findbase);
                usleep(1000*200);
                serviceClient_gripperClose.call(srv_closeGripper);
                usleep(1000*500);
                if(sleep_once_count==0)
                {
                    sleep(2);
                    Hsc3apiInstance::getInstance()->setR(41,1.0);
                }
//                Hsc3apiInstance::getInstance()->setR(40,0.0);
                sleep_once_count++;
                cout<<sleep_once_count<<endl;

            } else{
                sleep_once_count=0;
            }
        }
        usleep(1000*500);
        cout<<"线程退出"<<endl;
    });
    t1.detach();

    if(hsc3RobotMove->RobPickAction()!=0){
        cout<<"RobPickAction 故障 "<<endl;
        setTaskState("Err");
        return;
    }
    stop_flag= true;
    outSwapModule->notifyGrabBaskectFinish();
//    hsc3RobotMove->programRunQuit();
    cout<<"抓取完成"<<endl;
        if(smartToNav_forplace()!=0){
            cout<<"智能导航place点失败"<<endl;
            return;
        }

    //放置药品
    sleep(1);
    //掉头
    cout<<"掉头"<<endl;
    avg_turn180degree();
    sleep(1);

        //线程一直监听
        value1=0;
        value2=0;
        stop_flag= false;
        std::thread t2([&]{
            cout<<"thread"<<endl;
            int sleep_once_count=0;
            while (ros::ok()&&(!stop_flag))
            {
                Hsc3apiInstance::getInstance()->getR(42,value2); //等待通知夹爪张开
                if(value2==1.0)
                {
                    cout<<"打开夹爪！"<<endl;
                    serviceClient_gripperOpen.call(srv_openGripper);
                    Hsc3apiInstance::getInstance()->setR(43,1.0);
                    sleep(1);
                    stop_flag= true;
                    break;
                }
                usleep(1000*500);
            }
            cout<<"线程退出"<<endl;
        });
        t2.detach();
    if(hsc3RobotMove->RobPlaceAction(placepose_index)!=0){
        cout<<"RobPickAction 故障 "<<endl;
        setTaskState("Err");
        return;
    }
    stop_flag= true;

    cout<<"第"<<j<<"次进行完成 "<<endl;
    }
}






void AgvTransportFsmTask::Init_quiting(const std::vector<std::string> &args) {
//    while (!isEnd_init){
//        cout<<"等待退出"<<endl;
//        sleep(1);
//    }

}

void AgvTransportFsmTask::Init_toSelfCheck(const std::vector<std::string> &args) {
    setTaskState("SelfCheck");
}

void AgvTransportFsmTask::SelfCheck_initing(const std::vector<std::string> &args) {
    cout<<"SelfCheck_initing "<<endl;
    judgeErrOrExit();
    //1.检测机器人是否上使能
    hsc3RobotMove->init();
    //
    setTaskState("AgvMoveToPick");

}

void AgvTransportFsmTask::SelfCheck_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::SelfCheck_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::SelfCheck_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::AgvMoveToPick_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
   if(nav->moveTo("pick1")!=0){
       setTaskState("Err");
       cout<<"导航失败"<<endl;
       return;
   }
   cout<<"导航到pick1"<<endl;
    if(nav->moveTo("pick2")!=0){
        setTaskState("Err");
        cout<<"导航失败"<<endl;
        return;
    }
   cout<<"导航到pick2"<<endl;
   setTaskState("WaitForBasket");
}

void AgvTransportFsmTask::AgvMoveToPick_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::AgvMoveToPick_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::AgvMoveToPick_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::WaitForBasket_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    //等待抓取点药品篮子到位
    while (!outSwapModule->pickPoseHasBaskect())
    {
        if(isEixt)
        {
            setTaskState("Exit");
            return;
        }
        sleep(1);
        cout<<"等待篮子到位信号R[31]"<<endl;
    }
    setTaskState("DetectObjPose");
}

void AgvTransportFsmTask::WaitForBasket_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::WaitForBasket_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::WaitForBasket_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::DetectObjPose_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    //1.去到检测姿势点
    if(hsc3RobotMove->RobGoToDetectPose(11)!=0)
    {
        setTaskState("Err");
        return;
    }
    //2.目标检测
    for (int i = 0; i < 4; ++i) {
        if(visionDetection->detection("ar","Ar")==0)
        {
//            setTaskState("Err");
            break;
//            return;
        }else{
            sleep(1);
        }
    }
    vector<double > targetPose(6);
    cout<<"DetectObjPose_initing 获取结果"<<endl;
    if(visionDetection->getJointAngleResult(targetPose)!=0){
        cout<<"获取关节角错误"<<endl;
        setTaskState("Err");
        return;
    }
    cout<<"识别到目标点J[0]:"<<targetPose[0]<<endl;
    cout<<"识别到目标点J[1]:"<<targetPose[1]<<endl;
    cout<<"识别到目标点J[2]:"<<targetPose[2]<<endl;
    cout<<"识别到目标点J[3]:"<<targetPose[3]<<endl;
    cout<<"识别到目标点J[4]:"<<targetPose[4]<<endl;
    cout<<"识别到目标点J[5]:"<<targetPose[5]<<endl;
    hsc3RobotMove->setJR_jointpose(0,4,targetPose);
    setTaskState("RobPickBasket");

}

void AgvTransportFsmTask::DetectObjPose_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::DetectObjPose_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::DetectObjPose_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::RobPickBasket_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    if(hsc3RobotMove->RobPickAction()!=0){
        cout<<"RobPickAction 故障 "<<endl;
        setTaskState("Err");
        return;
    }
    outSwapModule->notifyGrabBaskectFinish();
    hsc3RobotMove->programRunQuit();
    cout<<"抓取完成"<<endl;
//    setTaskState("AgvMoveToPlace");


}

void AgvTransportFsmTask::RobPickBasket_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::RobPickBasket_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::RobPickBasket_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::AgvMoveToPlace_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    cout<<"准备去到palce点"<<endl;
    if(nav->moveTo("place")!=0){
        cout<<"导航到place点故障"<<endl;
        setTaskState("Err");
        return;
    }
    setTaskState("WaitForPlacePoseEmpty");

}

void AgvTransportFsmTask::AgvMoveToPlace_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::AgvMoveToPlace_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::AgvMoveToPlace_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::WaitForPlacePoseEmpty_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    while (!outSwapModule->placePoseHasClearBaskect()){
        if(isEixt)
        {
            setTaskState("Exit");
            return;
        }
        sleep(1);
        cout<<"放置台清空R[34] "<<endl;
    }
    setTaskState("RobPlaceBasket");

}

void AgvTransportFsmTask::WaitForPlacePoseEmpty_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::WaitForPlacePoseEmpty_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::WaitForPlacePoseEmpty_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::RobPlaceBasket_initing(const std::vector<std::string> &args) {
    judgeErrOrExit();
    cout<<"RobPlaceBasket_initing 状态"<<endl;
    if(hsc3RobotMove->RobPlaceAction(placepose_index)!=0){
        cout<<"RobPlaceAction故障"<<endl;
        setTaskState("Err");
    }
    outSwapModule->notifyPlaceBaskectFinish();
//    setTaskState("AgvMoveToPick");

}

void AgvTransportFsmTask::RobPlaceBasket_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::RobPlaceBasket_toExit(const std::vector<std::string> &args) {
    isEixt= true;
}

void AgvTransportFsmTask::RobPlaceBasket_toErr(const std::vector<std::string> &args) {
    isErr=true;
}

void AgvTransportFsmTask::Exit_initing(const std::vector<std::string> &args) {
    setTaskState("init");
}

void AgvTransportFsmTask::Exit_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::Err_initing(const std::vector<std::string> &args) {
    cout<<"进入故障状态"<<endl;
}

void AgvTransportFsmTask::Err_quiting(const std::vector<std::string> &args) {

}

void AgvTransportFsmTask::Err_toInit(const std::vector<std::string> &args) {
    setTaskState("init");
}

void AgvTransportFsmTask::publishStateMsg(bool status,std::string behevior,std::string meassage) {
    State sta;
    sta.status=status;
    sta.behevior=behevior;
    sta.meassage = {meassage};
    setRecallState(sta);
    notityRecall();
    usleep(1000);
}

void AgvTransportFsmTask::judgeErrOrExit(){
    if(isEixt){
        setTaskState("Exit");
        return;
    }
    if(isErr){
        setTaskState("Err");
        return;
    }
}

void AgvTransportFsmTask::avg_turn180degree() {
    int count=0;
    while(count<50*5){
        count++;
        usleep(1000*33);
        geometry_msgs::Twist msg;
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=-0.4;
        vel_pub.publish(msg);
//        cout<<"速度发布"<<endl;
    }
}

void AgvTransportFsmTask::agv_gostright() {
    geometry_msgs::Twist msg;
    msg.linear.x=0.2;
    msg.linear.y=0;
    msg.linear.z=0;
    msg.angular.x=0;
    msg.angular.y=0;
    msg.angular.z=0;
    int count=0;
    while(count<50*6){
        count++;
        usleep(1000*33);
        vel_pub.publish(msg);
//        cout<<"速度发布"<<endl;
    }
}



void AgvTransportFsmTask::callback_agvResult_subscriber(move_base_msgs::MoveBaseActionResult msg) {
    if(msg.status.status==3){
        reach_finish= true;
        cout<<"导航成功"<<endl;
    } else{
        reach_finish= false;
        cout<<"导航失败"<<endl;
    }
}

int AgvTransportFsmTask::navToPose(string pose) {
    //导航到place点
    cout<<"导航去"<<pose<<"点"<<endl;
//    nav->clearCostMap();
//    sleep(1);
//    nav->mappingIteration(10);
//    sleep(1);
    nav->clearCostMap();

    nav->moveTo(pose);
    //检查导航是否成功
    sleep(1);
    if(!reach_finish){
        cout<<"导航去"<<pose<<"点失败"<<endl;
        return -1;
    } else{
        reach_finish= false;
    }
    return 0;
}

int AgvTransportFsmTask::smartToNav_forpick(){
    bool sucess = false;
    int count=0;
    while(count<5){
        for (int i = 0; i <pick_poseList.size(); ++i) {
            if(navToPose(pick_poseList[i])==0){
                return 0;
            }
            nav->mappingIteration(10);
        }
//        //去导航抓取过渡点
//        navToPose("pass3");
        count++;
    }
    return -1;
}

int AgvTransportFsmTask::smartToNav_forplace(){
    bool sucess = false;
    int count=0;
    while(count<5){
        for (int i = 0; i <place_poseList.size(); ++i) {
            if(navToPose(place_poseList[i])==0){
                if(i==1)
                {
                    placepose_index=14;
                } else
                {
                    placepose_index=13;
                }
                return 0;
            }
            nav->mappingIteration(10);
        }
//        //去导航抓取过渡点
//        navToPose("pass4");
        count++;
    }
    return -1;
}

void AgvTransportFsmTask::callback_joyKey_sub_subscriber(sensor_msgs::Joy msg) {
    if(msg.buttons[3]==1 &&(!enable_flag)){
        cout<<"机器人下使能"<<endl;
        srv_enable.request.enable= false;
        robotEnable.call(srv_enable);
        enable_flag=true;
    } else{
        enable_flag= false;
    }
}

