#pragma once

#include <HsTaskFramework.h>
#include "rubik_cube_solve_func.h"
#include <functional>
#include <assert.h>
#include <ros/ros.h>
#include <atomic>

namespace HsFsm{
    class RubikCubeSolveTask : public HsTaskFramework 
    {
    public:
        RubikCubeSolveTask(const string& taskName);
        RubikCubeSolveTask(){}
        ~RubikCubeSolveTask();

        virtual void init();
        virtual void quit();
        virtual bool registerTaskList();

        std::atomic<bool> isRunInit;
    private:
        RubikCubeSolve* rubikPtr;
        void threadForSwithState(const char* state);
        void publishStateMsg(bool status, std::string behevior, std::string meassage);
        // 状态行为函数
        void init_initing(const std::vector<std::string>& args);
        void init_quiting(const std::vector<std::string>& args);
        void init_toHome(const std::vector<std::string>& args);
        void init_error(const std::vector<std::string>& args);
        void exit_initing(const std::vector<std::string>& args);
        void exit_quiting(const std::vector<std::string>& args);
        void home_initing(const std::vector<std::string>& args);
        void home_photo(const std::vector<std::string>& args);
        void home_error(const std::vector<std::string>& args);
        void home_toStep(const std::vector<std::string>& args);
        void home_toRecord(const std::vector<std::string>& args);
        void photo_initing(const std::vector<std::string>& args);
        void photo_request_data(const std::vector<std::string>& args);
        void photo_place(const std::vector<std::string>& args);
        void photo_error(const std::vector<std::string>& args);
        void request_data_initing(const std::vector<std::string>& args);
        void request_data_solve(const std::vector<std::string>& args);
        void request_data_place(const std::vector<std::string>& args);
        void request_data_error(const std::vector<std::string>& args);
        void solve_initing(const std::vector<std::string>& args);
        void solve_place(const std::vector<std::string>& args);
        void solve_error(const std::vector<std::string>& args);
        void place_initing(const std::vector<std::string>& args);
        void place_toHome(const std::vector<std::string>& args);
        void error_initing(const std::vector<std::string>& args);
        void step_initing(const std::vector<std::string>& args);
        void step_next(const std::vector<std::string>& args);
        void step_toHome(const std::vector<std::string>& args);
        void record_initing(const std::vector<std::string>& args);
        void record_next(const std::vector<std::string>& args);
    private:
        std::atomic<int> isAuto;
    };
}