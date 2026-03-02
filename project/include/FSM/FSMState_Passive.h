/**********************************************************************
 * FSM 被动状态 (FSMState_Passive)
 * 
 * 职责：
 * 1. 它是机器人的“避风港”。在该状态下，电机所有指令设为 0。
 * 2. 它不执行任何主动控制，仅维持基本通信。
 * 3. 它负责监听外部信号（如键盘/遥控器），决定何时“唤醒”机器人进入起立状态。
***********************************************************************/
#ifndef FSMSTATE_PASSIVE_H
#define FSMSTATE_PASSIVE_H

#include "FSM/FSMState.h"

class FSMState_Passive : public FSMState {
    
public:
    FSMState_Passive(ControlFSMData* data);

    void enter() override;

    void run() override;

    void exit() override;

    /**
     * @brief 监听切换请求
     */
    FSMStateName checkTransition() override;
};

#endif // FSMSTATE_PASSIVE_H
