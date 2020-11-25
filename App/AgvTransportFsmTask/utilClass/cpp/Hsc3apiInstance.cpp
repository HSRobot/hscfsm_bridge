#include "Hsc3apiInstance.h"

shared_ptr<Hsc3apiInstance> Hsc3apiInstance::hsc3apiInstance = nullptr;
std::mutex Hsc3apiInstance::m_mutex;

Hsc3apiInstance::Hsc3apiInstance() {
    commapi = new CommApi();
    proMo = new ProxyMotion(commapi);
//    proIO = new ProxyIO(commapi);
//    proSys = new ProxySys(commapi);
    proVm = new ProxyVm(commapi);
    proVar = new ProxyVar(commapi);
    commapi->setAutoConn(false);//关闭自动重连功能，否则连接失败
}

Hsc3apiInstance::~Hsc3apiInstance() {
    delete commapi;
    delete proMo;
    commapi = nullptr;
    proMo = nullptr;
}

shared_ptr<Hsc3apiInstance> Hsc3apiInstance::getInstance() {
    if(hsc3apiInstance== nullptr)
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        if(hsc3apiInstance== nullptr)
        {
            hsc3apiInstance=shared_ptr<Hsc3apiInstance>(new Hsc3apiInstance);
        }
    }
    return hsc3apiInstance;
}

int Hsc3apiInstance::connect(const std::string &strIp, uint16_t uPort) {
    int ret=0;
    robotIp_=strIp;
    robotPort_=uPort;
    ret =commapi->connect(strIp, uPort);
    return ret;
}

bool Hsc3apiInstance::hsc3ReConnect() {
    int ret=0;
    if(!(commapi->isConnected())){
        ret = commapi->connect(robotIp_, (uint16_t)robotPort_);
        if(ret != 0){
            std::cout<<"connect hsr_robot faile,return value is "<<ret<<" !"<<std::endl;
        }
        return ret == 0 ? true : false;
    }
    return true;
}

int Hsc3apiInstance::setStartUpProject(string programName) {
    int ret=0;
    //判断是否重连成功
    if(!hsc3ReConnect()){
        return -1;
    }
    //判断是否上使能
    bool en;
    ret = proMo->getGpEn(0,en);
    if(ret != 0){
        return -2;
    }
    //加载程序
    const string path = "./script";

    // load program
    ret = proVm->load(path,programName);
    if(ret != 0){
        return -3;
    }
    this_thread::sleep_for(chrono::seconds(1));
    // start program
    ret = proVm->start(programName);
    if(ret != 0){
        return -4;
    }
    return 0;
}

int Hsc3apiInstance::setStopProject(string programName) {
    proVm->stop(programName);
    proVm->unload(programName);
    return 0;
}

int Hsc3apiInstance::getR(int32_t index, double& value) {
    int ret=0;
    if(!hsc3ReConnect()){
        return -1;
    }
    ret=proVar->getR(index,value);
    return ret;
}

int Hsc3apiInstance::setR(int32_t index, double value) {
    int ret=0;
    if(!hsc3ReConnect()){
        return -1;
    }
    ret=proVar->setR(index,value);
    return ret;
}

int Hsc3apiInstance::setJR(int8_t gpId, int32_t index, const JntPos &data) {
    int ret=0;
    if(!hsc3ReConnect()){
        return -1;
    }
    ret=proVar->setJR(gpId,index,data);
    return ret;
}



