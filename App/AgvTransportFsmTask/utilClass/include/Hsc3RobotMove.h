#ifndef CPP_WORK_HSC3ROBOTMOVE_H
#define CPP_WORK_HSC3ROBOTMOVE_H

#include "Hsc3apiInstance.h"

enum RValue{
    R_ProgramExit=10,
    R_DetectionAction=11,
    R_PickAction=12,
    R_PlaceAction=13,
    R_DetectionActionFinish=21,
    R_PickActionFinish=22,
    R_PlaceActionFinish=23
};

class Hsc3RobotMove {
public:
    Hsc3RobotMove();
    ~Hsc3RobotMove();

private:
    int clear_Rvalue();

public:

    /***
     * 初始化配置
     * @return
     */
    int init();

    /***
     * 退出正在运行的程序
     * @return
     */
    int programRunQuit();

    /***
     * 机器人去到检测点
     * @return
     */
    int RobGoToDetectPose(int index_pose);

    /***
     * 机器人抓取篮子动作
     * @return
     */
    int RobPickAction();

    /***
     * 机器人执行放置篮子动作
     * @return
     */
    int RobPlaceAction(int index_pose);

    int setJR_jointpose(int8_t gpId, int32_t index, const vector<double > & data);

};


#endif //CPP_WORK_HSC3ROBOTMOVE_H
