#include "OutSwapModule.h"

OutSwapModule::OutSwapModule() {

}

OutSwapModule::~OutSwapModule() {

}

bool OutSwapModule::pickPoseHasBaskect() {
    double value_R_BaskectReach=0.0;
    Hsc3apiInstance::getInstance()->getR(R_BaskectReach,value_R_BaskectReach);
    cout<<"获取到R[31]:"<<value_R_BaskectReach<<endl;
    return value_R_BaskectReach==1.0;
}

int OutSwapModule::notifyGrabBaskectFinish() {
    Hsc3apiInstance::getInstance()->setR(R_BaskectMoveAwayByRob,1.0);
    return 0;
}

bool OutSwapModule::placePoseHasClearBaskect() {
    double value_R_BaskectPlaceOver=0.0;
    Hsc3apiInstance::getInstance()->getR(R_BaskectPlacePoseClear,value_R_BaskectPlaceOver);
    cout<<"获取到R[34]:"<<value_R_BaskectPlaceOver<<endl;

    return value_R_BaskectPlaceOver==1.0;
}

int OutSwapModule::notifyPlaceBaskectFinish() {
    Hsc3apiInstance::getInstance()->setR(R_BaskectPlaceOver,1.0);
    return 0;
}

