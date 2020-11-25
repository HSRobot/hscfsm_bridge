#include "Hsc3RobotMove.h"

Hsc3RobotMove::Hsc3RobotMove() {

}

Hsc3RobotMove::~Hsc3RobotMove() {

}

int Hsc3RobotMove::init() {
    int ret=0;
    //设备连接
    ret=Hsc3apiInstance::getInstance()->connect("192.168.99.3",23234);
//    ret=0;
    if(ret!=0){
        cout<<"机器人连接失败 "<<endl;
        return -1;
    }
    //清理R寄存器的值
//    clear_Rvalue();
    //加载一次机器人程序
    ret = Hsc3apiInstance::getInstance()->setStartUpProject("PICKOBJ.PRG");
    if(ret!=0){
        cout<<"机器人程序启动失败"<<endl;
        return -2;
    }
    return 0;
}

int Hsc3RobotMove::programRunQuit() {
    Hsc3apiInstance::getInstance()->setR(R_ProgramExit,1.0);
    this_thread::sleep_for(chrono::seconds(1));
    Hsc3apiInstance::getInstance()->setStopProject("pickObj");
    cout<<"机器人程序退出完成"<<endl;
    return 0;
}

int Hsc3RobotMove::RobGoToDetectPose(int index_pose) {
    //改变R寄存器的值与机器人示教程序进行交互
    Hsc3apiInstance::getInstance()->setR(index_pose,1.0);
    this_thread::sleep_for(chrono::seconds(2));
    bool stop= false;
    double progRunFinish=0;
    int time_count=0;
    while (!stop){
        //获取寄存器的值，确保动作已经完成
        Hsc3apiInstance::getInstance()->getR(R_DetectionActionFinish,progRunFinish);
        cout<<"获取R[21]的值"<<progRunFinish<<endl;
        if(progRunFinish==1.0)
        {
            Hsc3apiInstance::getInstance()->setR(R_DetectionActionFinish,0.0);
            cout<<"检测到动作完成信号，程序退出"<<endl;
            stop= true;
            break;
        }
        //超时判断
        if(time_count>200)
        {
            cout<<"机器人去到拍照点失败"<<endl;
            return -1;
        }
        time_count++;
        this_thread::sleep_for(chrono::seconds(1));
    }
    return 0;
}

int Hsc3RobotMove::RobPickAction() {
    //改变R寄存器的值与机器人示教程序进行交互
    Hsc3apiInstance::getInstance()->setR(R_PickAction,1.0);
    bool stop= false;
    double progRunFinish=0;
    int time_count=0;
    while (!stop){
        //获取寄存器的值，确保动作已经完成
        Hsc3apiInstance::getInstance()->getR(R_PickActionFinish,progRunFinish);
        cout<<"获取R[22]的值"<<progRunFinish<<endl;
        if(progRunFinish==1.0)
        {
            Hsc3apiInstance::getInstance()->setR(R_PickActionFinish,0.0);
            cout<<"检测到动作完成信号，程序退出"<<endl;
            stop= true;
            break;
        }
        //超时判断
        if(time_count>100)
        {
            cout<<"机器人抓取失败"<<endl;
            return -1;
        }
        time_count++;
        this_thread::sleep_for(chrono::seconds(1));
    }
    return 0;
}

int Hsc3RobotMove::RobPlaceAction(int index_pose) {
    //改变R寄存器的值与机器人示教程序进行交互 13,14
    Hsc3apiInstance::getInstance()->setR(index_pose,1.0);
    bool stop= false;
    double progRunFinish=0;
    int time_count=0;
    while (!stop){
        //获取寄存器的值，确保动作已经完成
        Hsc3apiInstance::getInstance()->getR(R_PlaceActionFinish,progRunFinish);
        cout<<"获取R[23]的值"<<progRunFinish<<endl;

        if(progRunFinish==1.0)
        {
            Hsc3apiInstance::getInstance()->setR(R_PlaceActionFinish,0.0);
            cout<<"检测到动作完成信号，程序退出 "<<endl;
            stop= true;
            break;
        }
        //超时判断
        if(time_count>200)
        {
            cout<<"机器人放置失败"<<endl;
            return -1;
        }
        time_count++;
        this_thread::sleep_for(chrono::seconds(1));
    }
    return 0;
}

int Hsc3RobotMove::clear_Rvalue() {

    for (int i = 10; i <15; ++i) {
        Hsc3apiInstance::getInstance()->setR(i,0.0);
    }

    for (int j = 20; j <25; ++j) {
        Hsc3apiInstance::getInstance()->setR(j,0.0);
    }
    return 0;
}

int Hsc3RobotMove::setJR_jointpose(int8_t gpId, int32_t index, const vector<double> &data) {
    JntPos jntPos;
    jntPos.vecPos=data;
    Hsc3apiInstance::getInstance()->setJR(gpId,index,jntPos);
    return 0;
}



