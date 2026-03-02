#include "FSM/FSMState_FixedStand.h"
#include "common/robotConfig.h"
#include <iostream>

FSMState_FixedStand::FSMState_FixedStand(ControlFSMData* data)
    : FSMState(data, FSMStateName::FIXEDSTAND, "FIXEDSTAND") {}

void FSMState_FixedStand::enter() {
    std::cout << "[FSM] 进入 FIXEDSTAND 状态, 开始准备起立..." << std::endl;
    _counter = 0;

    // 1. 捕捉当前 12 个关节的角度作为起始位置
    for(int i=0; i<4; i++) {
        _startJointAngles[i] = _legCtrl->data[i].q;
    }

    // 2. 设定目标位置 (使用配置文件中的 STAND_HEIGHT)
    Vec3 targetFootPos(0.0, 0.0, -STAND_HEIGHT); 
    
    for(int i=0; i<4; i++) {
        if(!_legCtrl->computeIK(i, targetFootPos, _targetJointAngles[i])) {
            std::cerr << "[错误] 腿 " << i << " 目标起立位置超出运动范围!" << std::endl;
        }
    }
}

void FSMState_FixedStand::run() {
    _counter += (1.0f / (float)CONTROL_FREQ);
    float ratio = _counter / _duration; // 计算进度系数 [0, 1]

    for(int i=0; i<4; i++) {
        // 3. 关节线性插值
        Vec3 q_cmd = jointLinearInterp(_startJointAngles[i], _targetJointAngles[i], ratio);
        
        // 4. 下发给 LegController
        _legCtrl->commands[i].q = q_cmd;
        _legCtrl->commands[i].dq.setZero();
        _legCtrl->commands[i].tau.setZero();
        _legCtrl->commands[i].kp = _stiff_kp;
        _legCtrl->commands[i].kd = _stiff_kd;
    }
}

FSMStateName FSMState_FixedStand::checkTransition() {
    // 如果起立动作执行完毕，可以根据用户输入切换到其他状态（暂时保持站立）
    if(_counter >= _duration + 0.5f) {
        // 后续可以增加遥控器按键切换逻辑
    }
    return FSMStateName::FIXEDSTAND;
}

void FSMState_FixedStand::exit() {
    std::cout << "[FSM] 退出 FIXEDSTAND 状态" << std::endl;
}
