#ifndef CPP_WORK_OUTSWAPMODULE_H
#define CPP_WORK_OUTSWAPMODULE_H
#include "Hsc3apiInstance.h"
#include <iostream>

using namespace std;

enum R_Value{
    R_BaskectReach=31,
    R_BaskectMoveAwayByRob=32,
    R_BaskectPlaceOver=33,
    R_BaskectPlacePoseClear=34,
};

class OutSwapModule {
public:
    OutSwapModule();
    ~OutSwapModule();
private:

public:
    /***
     * 抓取点是否有药品篮子
     * @return
     */
    bool pickPoseHasBaskect();

    /***
     * 通知抓取篮子完成
     * @return
     */
    int notifyGrabBaskectFinish();

    /***
     * 判断抓取点是否已经清理了篮子
     * @return
     */
    bool placePoseHasClearBaskect();

    /**
     * 通知药品篮子放置完成
     * @return
     */
    int notifyPlaceBaskectFinish();

};


#endif //CPP_WORK_OUTSWAPMODULE_H
