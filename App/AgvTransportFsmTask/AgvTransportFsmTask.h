#ifndef HSCFSM_BRIDGE_AGVTRANSPORTFSMTASK_H
#define HSCFSM_BRIDGE_AGVTRANSPORTFSMTASK_H

#include "Hsc3RobotMove.h"
#include "navigation.h"
#include "OutSwapModule.h"
#include "vision_detection.h"
#include "Hsc3apiInstance.h"
#include <HsTaskFramework.h>

#include <functional>
#include <assert.h>
#include <ros/ros.h>

#include "hsr_gripper_driver/serial_open_srv.h"
#include "hsr_gripper_driver/close_srv.h"
#include "hsr_gripper_driver/open_srv.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "sensor_msgs/Joy.h"
#include "hsr_rosi_device/SetEnableSrv.h"

namespace HsFsm {
    class AgvTransportFsmTask : public HsTaskFramework
    {
    public:
        AgvTransportFsmTask();
        AgvTransportFsmTask(const string &taskName);

        /**
         * @brief init
         */
        virtual void init();

        /**
         * @brief quit
         */
        virtual void quit();

        /**
         * @brief registerTaskList
         * @return
         */
        virtual bool registerTaskList();

    private:
        Navigation* nav;
        VisionDetection* visionDetection;
        Hsc3RobotMove* hsc3RobotMove;
        OutSwapModule* outSwapModule;

        atomic<bool > reach_finish;
//        atomic<bool > isEnd_SelfCheck;
        atomic<bool > isEnd_init;
        atomic<bool > isErr;
        atomic<bool > isEixt;
        atomic<bool > enable_flag;
        atomic<int > placepose_index;

        ros::Publisher vel_pub;
        ros::Subscriber goalResult_sub;
        ros::Subscriber joyKey_sub;

        ros::ServiceClient serviceClient_serialOpen;
        ros::ServiceClient serviceClient_gripperOpen;
        ros::ServiceClient serviceClient_findbase;
        ros::ServiceClient serviceClient_gripperClose;
        ros::ServiceClient robotEnable;

        hsr_gripper_driver::serial_open_srv srv_serial_open;
        hsr_gripper_driver::close_srv srv_closeGripper;
        hsr_gripper_driver::open_srv srv_openGripper;
        hsr_gripper_driver::open_srv srv_findbase;

        hsr_rosi_device::SetEnableSrv srv_enable;

        vector<string > pick_poseList;
        vector<string > place_poseList;

    private:
        void publishStateMsg(bool status,std::string behevior,std::string meassage);
        void judgeErrOrExit();
        void avg_turn180degree();
        void agv_gostright();
        int navToPose(string pose);
        int smartToNav_forpick();
        int smartToNav_forplace();

        void callback_agvResult_subscriber(move_base_msgs::MoveBaseActionResult msg);
        void callback_joyKey_sub_subscriber(sensor_msgs::Joy msg);
        //状态行为函数
        void Init_initing(const std::vector<std::string> &args);
        void Init_quiting(const std::vector<std::string> &args);
        void Init_toSelfCheck(const std::vector<std::string> &args);
        void SelfCheck_initing(const std::vector<std::string> &args);
        void SelfCheck_quiting(const std::vector<std::string> &args);
        void SelfCheck_toExit(const std::vector<std::string> &args);
        void SelfCheck_toErr(const std::vector<std::string> &args);
        void AgvMoveToPick_initing(const std::vector<std::string> &args);
        void AgvMoveToPick_quiting(const std::vector<std::string> &args);
        void AgvMoveToPick_toExit(const std::vector<std::string> &args);
        void AgvMoveToPick_toErr(const std::vector<std::string> &args);
        void WaitForBasket_initing(const std::vector<std::string> &args);
        void WaitForBasket_quiting(const std::vector<std::string> &args);
        void WaitForBasket_toExit(const std::vector<std::string> &args);
        void WaitForBasket_toErr(const std::vector<std::string> &args);
        void DetectObjPose_initing(const std::vector<std::string> &args);
        void DetectObjPose_quiting(const std::vector<std::string> &args);
        void DetectObjPose_toExit(const std::vector<std::string> &args);
        void DetectObjPose_toErr(const std::vector<std::string> &args);
        void RobPickBasket_initing(const std::vector<std::string> &args);
        void RobPickBasket_quiting(const std::vector<std::string> &args);
        void RobPickBasket_toExit(const std::vector<std::string> &args);
        void RobPickBasket_toErr(const std::vector<std::string> &args);
        void AgvMoveToPlace_initing(const std::vector<std::string> &args);
        void AgvMoveToPlace_quiting(const std::vector<std::string> &args);
        void AgvMoveToPlace_toExit(const std::vector<std::string> &args);
        void AgvMoveToPlace_toErr(const std::vector<std::string> &args);
        void WaitForPlacePoseEmpty_initing(const std::vector<std::string> &args);
        void WaitForPlacePoseEmpty_quiting(const std::vector<std::string> &args);
        void WaitForPlacePoseEmpty_toExit(const std::vector<std::string> &args);
        void WaitForPlacePoseEmpty_toErr(const std::vector<std::string> &args);
        void RobPlaceBasket_initing(const std::vector<std::string> &args);
        void RobPlaceBasket_quiting(const std::vector<std::string> &args);
        void RobPlaceBasket_toExit(const std::vector<std::string> &args);
        void RobPlaceBasket_toErr(const std::vector<std::string> &args);
        void Exit_initing(const std::vector<std::string> &args);
        void Exit_quiting(const std::vector<std::string> &args);
        void Err_initing(const std::vector<std::string> &args);
        void Err_quiting(const std::vector<std::string> &args);
        void Err_toInit(const std::vector<std::string> &args);
        void loop_initing(const std::vector<std::string> &args);

    };


}


#endif //HSCFSM_BRIDGE_AGVTRANSPORTFSMTASK_H
