#include "FSM/FSMState_Passive.h"
#include <iostream>

FSMState_Passive::FSMState_Passive(ControlFSMData* data)
    : FSMState(data, FSMStateName::PASSIVE, "PASSIVE") {}

void FSMState_Passive::enter() {
    std::cout << "[FSM] PASSIVE: 进入被动模式, 电机已设为安全阻尼" << std::endl;
}

void FSMState_Passive::run() {
    // PASSIVE 模式下不主动控制，仅维持基本指令
    for(int i=0; i<4; i++) {
        // 让期望位置等于当前位置，避免突变
        _data->legController->commands[i].q = _data->legController->data[i].q;
        _data->legController->commands[i].dq.setZero();
        _data->legController->commands[i].tau.setZero();
        _data->legController->commands[i].kp = 0.0f;
        _data->legController->commands[i].kd = 3.0f; // 给予一定旋转阻尼(Damping)
    }
}

void FSMState_Passive::exit() {
    std::cout << "[FSM] PASSIVE: 退出被动模式" << std::endl;
}

FSMStateName FSMState_Passive::checkTransition() {
    // 实际逻辑中这里会检测遥控器/键盘
    return FSMStateName::INVALID; 
}