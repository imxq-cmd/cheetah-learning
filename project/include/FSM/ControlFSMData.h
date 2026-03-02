#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "control/LegController.h"
#include "interface/IOInterface.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

/**
 * ControlFSMData
 * 这是一个数据容器，用于在 FSM 状态机和各个状态之间传递核心资源的指针。
 * 这样每个状态都可以访问到底层硬件、运动学控制器和通信数据包。
 */
struct ControlFSMData {
    LegController* legController;
    LowlevelState* lowState;
    LowlevelCmd* lowCmd;
    IOInterface* ioInterface;
};

#endif // CONTROLFSMDATA_H
