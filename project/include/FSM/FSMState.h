/**********************************************************************
 * FSM 状态抽象基类 (FSMState)
 * 
 * 所有具体的机器人控制器 (如 Stand, Trot) 都需要继承这个类并实现其生命周期函数。
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include "common/enumClass.h"
#include "control/LegController.h"
#include "FSM/ControlFSMData.h"

class FSMState {
public:
    FSMState(ControlFSMData* data, FSMStateName stateName, std::string stateNameStr)
        : _data(data), _stateName(stateName), _stateNameStr(stateNameStr) {
            _legCtrl = data->legController;
        }
    
    virtual ~FSMState() {}

    /**
     * @brief 进入该状态时执行一次
     */
    virtual void enter() = 0;

    /**
     * @brief 周期性循环执行的核心控制逻辑 (由状态机每帧调用)
     */
    virtual void run() = 0;

    /**
     * @brief 离开该状态时执行一次
     */
    virtual void exit() = 0;

    /**
     * @brief 检查是否需要切换到其他状态 (由状态机自动轮询)
     * @return 如果返回 INVALID 则不切换，否则返回目标状态名
     */
    virtual FSMStateName checkTransition() { return FSMStateName::INVALID; }

    // 获取基本属性
    FSMStateName stateName() const { return _stateName; }
    std::string stateNameStr() const { return _stateNameStr; }

protected:
    ControlFSMData* _data;
    LegController* _legCtrl;  // 状态机共享的腿部控制器指针 (快捷方式)
    FSMStateName _stateName;  // 本状态枚举名
    std::string _stateNameStr; // 本状态字符串名
};

#endif // FSMSTATE_H
