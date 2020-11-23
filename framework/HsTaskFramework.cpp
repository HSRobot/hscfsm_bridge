#include "HsTaskFramework.h"
using namespace HsFsm;
#include <functional>

using namespace std;
//std::shared_ptr<semaphore> HsTaskFramework::notitySem = nullptr;
HsFsm::State HsTaskFramework::currentState;
HsTaskFramework::HsTaskFramework()
{
//    fsmStack = std::make_shared<fsm::stack>();
//    threadpool = std::make_shared<ThreadPool>(2);
}

HsTaskFramework::HsTaskFramework(ros::NodeHandle &nh, std::shared_ptr<HsTaskFramework> &fsm):typeCode(0),taskRunStatus(false)
{
    this->nh = nh;
    this->framework = fsm;

    fsm->fsmStack = std::make_shared<fsm::stack>();
    this->fsmStack = fsm->fsmStack;

    fsm->notitySem = std::make_shared<semaphore>(fsm->taskName);
    this->notitySem = fsm->notitySem;

    this->threadpool = std::make_shared<ThreadPool>(4);
    fsm->threadpool = threadpool;

    this->mode = Auto;
    fsm->mode = mode;

    this->taskName = fsm->taskName;
}

void HsTaskFramework::init()
{

    framework->init();
    setInitState();

}
/**
 * @brief HsTaskFramework::setCommand 指令的下发异步
 * @param cmd
 * @return
 */
bool HsTaskFramework::setCommand(const CmdInputData &cmd)
{
    //
//    framework->setCommand(cmd);
    setCommandProxy(cmd);
}

void HsTaskFramework::quit()
{
    framework->quit();
    setExitingAction();

}

State HsTaskFramework::getState()
{
    return getTaskState();
}

bool HsTaskFramework::registerTaskList()
{
    return framework->registerTaskList();
}

void HsTaskFramework::waitRecall()
{
    notitySem->wait();
}

string HsTaskFramework::getTaskName()
{
    return taskName;
}

void HsTaskFramework::debugTaskList()
{
    std::cout <<this->fsmStack.get();
}

void HsTaskFramework::setMode(Mode mode)
{
    this->mode = mode;
}

/**
 * @brief HsTaskFramework::setCommandProxy 指令运行代理函数
 * @param behevior
 * @return
 */
bool HsTaskFramework::setCommandProxy(const CmdInputData &cmd)
{
    if(!fsmStack->findFuntionalIsVaild(cmd.baheviror.c_str()))
    {
        taskRunStatus = false;
        typeCode = -1;
        notityRecall();
        return false;
    }

    std::cout << cmd;
    threadpool->enqueue(&fsm::stack::commandThread, fsmStack.get(), cmd.baheviror.c_str(),cmd.param);
    return true;

}


void HsTaskFramework::notityRecall()
{
    notitySem->signal();
}

/**
 * @brief HsFsm::HsTaskFramework::registerTask
 * @param state
 * @param behevior
 * @param call
 */
void HsFsm::HsTaskFramework::registerTask( HsFsm::state state, HsFsm::action behevior, HsFsm::call call)
{

    fsmStack->on(string(state), string(behevior)) = call;
}

State HsTaskFramework::getTaskState()
{
     fsm::state ret = fsmStack->get_state();
     State current;
     current.meassage = ret.args;
     current.stateName = ret.name;
     current.Type = currentState.Type;
     current.status = currentState.status;
     current.meassage = currentState.meassage;
     current.behevior = currentState.behevior;
     return current;
}

void HsTaskFramework::setTaskState(HsFsm::state state)
{

    switch (mode) {
    case Auto:{
        threadpool->enqueue(&fsm::stack::set, fsmStack.get(), state );
        break;
    }
    case Manual:
    {
        string current = string(state);
        if(current == "init" ){
            break;
        }
        threadpool->enqueue(&fsm::stack::set, fsmStack.get(), "Middle" );
     }
        break;
    default:
        break;
    }


}

void HsTaskFramework::setInitState()
{
    threadpool->Start();
    setTaskState("init");
//    fsmStack->set("init");
}

void HsTaskFramework::setExitingAction()
{
//    fsmStack->set("quit");
    setTaskState("quit");

    threadpool->Stop();
}

/**
 * @brief HsTaskFramework::setRecallState 设置返回view 前端的数据信息
 * @param state
 */
void HsTaskFramework::setRecallState(State& state)
{
    this->currentState = state;
    notityRecall();
}

ros::NodeHandle *HsTaskFramework::getRosHandler()
{
    return &nh;
}

H_EXPORT_PLUGIN(HsTaskFramework, "HsTaskFramework" , "1.0")

