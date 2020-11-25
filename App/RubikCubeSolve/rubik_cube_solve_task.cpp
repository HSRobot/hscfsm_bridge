#include "rubik_cube_solve_task.h"
using namespace HsFsm;

RubikCubeSolveTask::RubikCubeSolveTask(const string& taskName)
{
    this->taskName = taskName;

}

RubikCubeSolveTask::~RubikCubeSolveTask()
{

}

void RubikCubeSolveTask::init()
{
    ROS_INFO("init state");
}

void RubikCubeSolveTask::quit()
{
    ROS_INFO("quit state");
}

bool RubikCubeSolveTask::registerTaskList()
{
    auto func_init_initing = std::bind(&RubikCubeSolveTask::init_initing, this, placeholders::_1);
    auto func_init_quiting = std::bind(&RubikCubeSolveTask::init_quiting, this, placeholders::_1);
    auto func_init_toHome = std::bind(&RubikCubeSolveTask::init_toHome, this, placeholders::_1);
    auto func_init_error = std::bind(&RubikCubeSolveTask::init_error, this, placeholders::_1);
    auto func_exit_initing = std::bind(&RubikCubeSolveTask::exit_initing, this, placeholders::_1);
    auto func_exit_quiting = std::bind(&RubikCubeSolveTask::exit_quiting, this, placeholders::_1);
    auto func_home_initing = std::bind(&RubikCubeSolveTask::home_initing, this, placeholders::_1);
    auto func_home_photo = std::bind(&RubikCubeSolveTask::home_photo, this, placeholders::_1);
    auto func_home_error = std::bind(&RubikCubeSolveTask::home_error, this, placeholders::_1);
    auto func_home_toStep = std::bind(&RubikCubeSolveTask::home_toStep, this, placeholders::_1);
    auto func_home_toRecord = std::bind(&RubikCubeSolveTask::home_toRecord, this, placeholders::_1);
    auto func_photo_initing = std::bind(&RubikCubeSolveTask::photo_initing, this, placeholders::_1);
    auto func_photo_request_data = std::bind(&RubikCubeSolveTask::photo_request_data, this, placeholders::_1);
    auto func_photo_place = std::bind(&RubikCubeSolveTask::photo_place, this, placeholders::_1);
    auto func_photo_error = std::bind(&RubikCubeSolveTask::photo_error, this, placeholders::_1);
    auto func_request_data_initing = std::bind(&RubikCubeSolveTask::request_data_initing, this, placeholders::_1);
    auto func_request_data_solve = std::bind(&RubikCubeSolveTask::request_data_solve, this, placeholders::_1);
    auto func_request_data_place = std::bind(&RubikCubeSolveTask::request_data_place, this, placeholders::_1);
    auto func_request_data_error = std::bind(&RubikCubeSolveTask::request_data_error, this, placeholders::_1);
    auto func_solve_initing = std::bind(&RubikCubeSolveTask::solve_initing, this, placeholders::_1);
    auto func_solve_place = std::bind(&RubikCubeSolveTask::solve_place, this, placeholders::_1);
    auto func_solve_error = std::bind(&RubikCubeSolveTask::solve_error, this, placeholders::_1);
    auto func_place_initing = std::bind(&RubikCubeSolveTask::place_initing, this, placeholders::_1);
    auto func_place_toHome = std::bind(&RubikCubeSolveTask::place_toHome, this, placeholders::_1);
    auto func_error_initing = std::bind(&RubikCubeSolveTask::error_initing, this, placeholders::_1);
    auto func_step_initing = std::bind(&RubikCubeSolveTask::step_initing, this, placeholders::_1);
    auto func_step_next = std::bind(&RubikCubeSolveTask::step_next, this, placeholders::_1);
    auto func_step_toHome = std::bind(&RubikCubeSolveTask::step_toHome, this, placeholders::_1);
    auto func_record_initing = std::bind(&RubikCubeSolveTask::record_initing, this, placeholders::_1);
    auto func_record_next = std::bind(&RubikCubeSolveTask::record_next, this, placeholders::_1);

    try
    {
        registerTask("init", "initing", func_init_initing);
        registerTask("init", "quiting", func_init_quiting);
        registerTask("init", "toHome", func_init_toHome);
        registerTask("init", "error", func_init_error);
        registerTask("exit", "initing", func_exit_initing);
        registerTask("exit", "quiting", func_exit_quiting);
        registerTask("home", "initing", func_home_initing);
        registerTask("home", "photo", func_home_photo);
        registerTask("home", "error", func_home_error);
        registerTask("home", "toStep", func_home_toStep);
        registerTask("home", "toRecord", func_home_toRecord);
        registerTask("photo", "initing", func_photo_initing);
        registerTask("photo", "request_data", func_photo_request_data);
        registerTask("photo", "place", func_photo_place);
        registerTask("photo", "error", func_photo_error);
        registerTask("request_data", "initing", func_request_data_initing);
        registerTask("request_data", "solve", func_request_data_solve);
        registerTask("request_data", "place", func_request_data_place);
        registerTask("request_data", "error", func_request_data_error);
        registerTask("solve", "initing", func_solve_initing);
        registerTask("solve", "place", func_solve_place);
        registerTask("solve", "error", func_solve_error);
        registerTask("place", "initing", func_place_initing);
        registerTask("place", "toHome", func_place_toHome);
        registerTask("error", "initing", func_error_initing);
        registerTask("step", "initing", func_step_initing);
        registerTask("step", "next", func_step_next);
        registerTask("step", "toHome", func_step_toHome);
        registerTask("record", "initing", func_record_initing);
        registerTask("record", "next", func_record_next);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}

void RubikCubeSolveTask::threadForSwithState(const char* state)
{
    setTaskState(state);
}

void RubikCubeSolveTask::publishStateMsg(bool status, std::string behevior, std::string meassage)
{
    State sta;
    sta.status = status;
    sta.behevior = behevior;
    sta.meassage = {meassage};
    setRecallState(sta);
    notityRecall();
    usleep(1000);
}


void RubikCubeSolveTask::init_initing(const std::vector<std::string>& args)
{
    isRunInit = true;
    ROS_INFO("rubik state: init, task initing");
    publishStateMsg(true, "initing", "-----init_initing------");
    ros::NodeHandle nh;
    rubikPtr = new RubikCubeSolve(nh);
    typeCode = 0;
    taskRunStatus = true;
    isRunInit = false;
}

void RubikCubeSolveTask::init_quiting(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: init, task quiting");
    publishStateMsg(true, "quiting", "-----init_quiting------");
    while (isRunInit)
    {
        usleep(10);
    }
}

void RubikCubeSolveTask::init_toHome(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: init, stask: tohome");
    for(auto i: args)
    {
        ROS_INFO_STREAM("i: " << i);
    }
    publishStateMsg(true, "quiting", "-----init_quiting------");
    setTaskState("home");
}

void RubikCubeSolveTask::init_error(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: init, stask: error");
    publishStateMsg(true, "error", "-----init_error------");
    setTaskState("error");
}

void RubikCubeSolveTask::exit_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: exit, task initing");
    publishStateMsg(true,"initing","-----exit_initing------");
    exit(-1);
}

void RubikCubeSolveTask::exit_quiting(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: exit, task quiting");
    publishStateMsg(true,"quiting","-----exit_quiting------");
}

void RubikCubeSolveTask::home_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: home, task initing");
    publishStateMsg(true, "initing", "-----home_initing------");
    if(rubikPtr->backHome(0) != 0 || rubikPtr->backHome(1) !=0)
    {
        setTaskState("error");
    }
}

void RubikCubeSolveTask::home_photo(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: home, task photo");
    publishStateMsg(true, "photo", "-----home_photo------");
    isAuto = 0;
    if(args.size() == 1)
        isAuto = std::atoi(args[0].c_str());
    setTaskState("photo");
}

void RubikCubeSolveTask::home_error(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: home, task error");
    publishStateMsg(true, "error", "-----home_error------");
    setTaskState("error");
}

void RubikCubeSolveTask::home_toStep(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: home, task toStep");
    publishStateMsg(true, "toStep", "-----home_toStep------");
    setTaskState("step");
}

void RubikCubeSolveTask::home_toRecord(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: home, task toRecord");
    publishStateMsg(true, "toRecord", "-----home_toRecord------");
    setTaskState("record");
}


void RubikCubeSolveTask::photo_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: photo, task initing");
    publishStateMsg(true, "initing", "-----photo_initing------");
    if(rubikPtr->photograph() != 0)
    {
        setTaskState("error");
    }
    else if(isAuto)
    {
        setTaskState("request_data");
    }
}

void RubikCubeSolveTask::photo_request_data(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: photo, task request_data");
    publishStateMsg(true, "request_data", "-----photo_request_data------");
    setTaskState("request_data");
}

void RubikCubeSolveTask::photo_place(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: photo, task place");
    publishStateMsg(true, "place", "-----photo_place------");
    setTaskState("place");
}

void RubikCubeSolveTask::photo_error(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: photo, task error");
    publishStateMsg(true, "error", "-----photo_error------");
    setTaskState("error");
}


void RubikCubeSolveTask::request_data_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: request_data, task initing");
    publishStateMsg(true, "initing", "-----request_data_initing------");
    if(rubikPtr->requestData() != 0)
    {
        setTaskState("error");
    }
    else if(isAuto)
    {
        setTaskState("solve");
    }
}

void RubikCubeSolveTask::request_data_solve(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: request_data, task solve");
    publishStateMsg(true, "solve", "-----request_data_solve------");
    setTaskState("solve");
}

void RubikCubeSolveTask::request_data_place(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: request_data, task place");
    publishStateMsg(true, "place", "-----request_data_place------");
    setTaskState("place");
}

void RubikCubeSolveTask::request_data_error(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: request_data, task error");
    publishStateMsg(true, "error", "-----request_data_error------");
    setTaskState("error");
}


void RubikCubeSolveTask::solve_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: solve, task initing");
    publishStateMsg(true, "initing", "-----solve_initing------");
    if(rubikPtr->solve() != 0)
    {
        setTaskState("error");
        return;
    }
    else if(isAuto)
    {
        setTaskState("place");
    }
}

void RubikCubeSolveTask::solve_place(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: solve, task place");
    publishStateMsg(true, "place", "-----solve_place------");
    setTaskState("place");
}

void RubikCubeSolveTask::solve_error(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: solve, task error");
    publishStateMsg(true, "error", "-----solve_error------");
    setTaskState("error");
}


void RubikCubeSolveTask::place_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: place, task initing");
    publishStateMsg(true, "initing", "-----place_initing------");
    if(rubikPtr->placeCube() != 0)
    {
        setTaskState("error");
        return;
    }
    else if (isAuto)
    {
        setTaskState("home");
    }
}

void RubikCubeSolveTask::place_toHome(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: place, task toHome");
    publishStateMsg(true, "toHome", "-----place_toHome------");
    setTaskState("home");
}

void RubikCubeSolveTask::error_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: error, task initing");
    publishStateMsg(true, "initing", "-----error_initing------");
    setTaskState("exit");
}

void RubikCubeSolveTask::step_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: step, task initing");
    publishStateMsg(true, "initing", "-----step_initing------");
    rubikPtr->InitializationState();
    rubikPtr->goPreparePose();
}

void RubikCubeSolveTask::step_next(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: step, task next");
    publishStateMsg(true, "next", "-----step_next------");
    int face = std::atoi(args[0].c_str());
    int angle = std::atoi(args[1].c_str());
    rubikPtr->analyseData(face, angle);
    if(rubikPtr->action() != 0)
    {
        setTaskState("error");
    }
}

void RubikCubeSolveTask::step_toHome(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: step, task toHome");
    publishStateMsg(true, "toHome", "-----step_toHome------");
    setTaskState("home");
}

void RubikCubeSolveTask::record_initing(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: record, task initing");
    publishStateMsg(true, "initing", "-----record_initing------");
}

void RubikCubeSolveTask::record_next(const std::vector<std::string>& args)
{
    ROS_INFO("rubik state: record, task next");
    publishStateMsg(true, "initing", "-----record_next------");
    rubikPtr->magicMoveToPointFSM(args);
}
