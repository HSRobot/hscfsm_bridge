#ifndef CPP_WORK_HSC3APIINSTANCE_H
#define CPP_WORK_HSC3APIINSTANCE_H
#include "CommApi.h"
#include "Hsc3Def.h"
#include "proxy/ProxyMotion.h"
#include "proxy/ProxyIO.h"
#include "proxy/ProxySys.h"
#include "proxy/ProxyVm.h"
#include "proxy/ProxyVar.h"
#include <iostream>
#include <atomic>
#include <mutex>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
using namespace std;
using namespace Hsc3::Comm;
using namespace Hsc3::Proxy;

class Hsc3apiInstance {
private:
    Hsc3apiInstance();
    static shared_ptr<Hsc3apiInstance> hsc3apiInstance;
    static std::mutex m_mutex;
    CommApi *commapi;
    ProxyMotion *proMo;
//    ProxyIO *proIO;
//    ProxySys *proSys;
    ProxyVm * proVm;
    ProxyVar *proVar;

    std::string robotIp_;
    int robotPort_;
    mutex locker;
    mutex locker1;
public:
    ~Hsc3apiInstance();

    /***
     * 获取单例对象指针
     * @return
     */
    static shared_ptr<Hsc3apiInstance> getInstance();

    /***
     * 机器人连接
     * @param strIp ip地址
     * @param uPort 端口号
     * @return
     */
    int connect(const std::string & strIp, uint16_t uPort);

    /***
     * 机器人重新连接
     * @return
     */
    bool hsc3ReConnect();

    /***
     * 启动程序
     * @param programName 程序名
     * @return
     */
    int setStartUpProject(string programName);

    /***
     * 停止程序
     * @param programName 程序名
     * @return
     */
    int setStopProject(string programName);

    /***
     * 获取R寄存器的值
     * @param index
     * @param value
     * @return
     */
    int getR(int32_t index, double& value);

    /***
     * 设置R寄存器的值
     * @param index
     * @param value
     * @return
     */
    int setR(int32_t index, double value);

    int setJR(int8_t gpId, int32_t index, const JntPos & data);

};


#endif //CPP_WORK_HSC3APIINSTANCE_H
