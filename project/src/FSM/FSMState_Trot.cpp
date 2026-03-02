#include "FSM/FSMState_Trot.h"
#include "common/robotConfig.h"
#include <iostream>

FSMState_Trot::FSMState_Trot(ControlFSMData* data)
    : FSMState(data, FSMStateName::TROT, "TROT") {
    
    // 初始化波形发生器 (周期 0.5s, 占比 0.5)
    _waveGait = new WaveGenerator(GRC_TROT_DURATION, 0.5, Vec4::Zero());
    
    _v_cmd.setZero();
    _v_actual.setZero();
}

FSMState_Trot::~FSMState_Trot() {
    delete _waveGait;
}

void FSMState_Trot::enter() {
    std::cout << "[FSM] 进入 TROT 步态模式" << std::endl;
    
    // 1. 初始化步态时钟 (重置相位)
    _waveGait->restart(); 

    // 2. 设定初始指令速度 (例如前进 0.2 m/s)
    _v_cmd << 0.2, 0.0, 0.0; 
    _v_actual = _v_cmd; // 暂时假设完美跟踪

    // 3. 记录初始足端位置
    for(int i=0; i<4; i++) {
        _pStart[i] = _legCtrl->data[i].p;
        _pEnd[i] = _pStart[i];
    }
}

void FSMState_Trot::run() {
    float dt = 1.0f / (float)CONTROL_FREQ;
    _waveGait->update(dt);

    for(int i=0; i<4; i++) {
        float status = _waveGait->getStatus(i); // 1:支撑, 0:摆动
        float phi = _waveGait->getPhase(i);     // 该阶段进度 [0, 1]

        Vec3 p_des;

        if(status > 0.5) { // --- 支撑相 (Stance) ---
            // 在简单的 Trot 实现中，支撑腿相对于身体向后移动
            // 这里我们让足端在 X 方向上线性跟随机体速度反方向运动
            p_des = _pStart[i]; // 以起跳点为基准
            p_des.x() -= _v_actual.x() * (phi * GRC_TROT_DURATION * 0.5f);
            p_des.y() -= _v_actual.y() * (phi * GRC_TROT_DURATION * 0.5f);
            p_des.z() = -STAND_HEIGHT; // 保持站立高度
        } 
        else { // --- 摆动相 (Swing) ---
            // 1. 如果刚进入摆动相 (phi 接近 0)，更新一次落脚点预测
            if(phi < 0.1) {
                _pStart[i] = _legCtrl->data[i].p;
                _calculateRaibertLP(i);
                _footTrajectory[i].setFootPos(_pStart[i], _pEnd[i]);
            }
            
            // 2. 利用摆线轨迹获取当前期望坐标
            p_des = _footTrajectory[i].getFootPos(phi, SWING_HEIGHT);
        }

        // 3. 逆运动学求解并下发
        _legCtrl->commands[i].pDes = p_des; // 记录期望位置
        _legCtrl->computeIK(i, p_des, _legCtrl->commands[i].q);
        
        // 设置 Trot 的典型增益 (摆动腿通常需要更高的 Kp)
        _legCtrl->commands[i].kp = 80.0f;
        _legCtrl->commands[i].kd = 3.0f;
    }
}

void FSMState_Trot::_calculateRaibertLP(int i) {
    float T_stance = GRC_TROT_DURATION * 0.5f;
    
    // Raibert 启发式落脚点公式
    // P_end = v_actual * T_stance / 2 + K * (v_actual - v_cmd)
    float k_recovery = 0.03f; // 速度纠偏增益

    // 计算相对于髋关节中心的中立偏移 (脚踩在肩膀正下方)
    // 注意：这里 _pEnd 为相对于髋关节坐标系的矢量
    _pEnd[i].x() = (T_stance / 2.0f) * _v_actual.x() + k_recovery * (_v_actual.x() - _v_cmd.x());
    _pEnd[i].y() = (T_stance / 2.0f) * _v_actual.y() + k_recovery * (_v_actual.y() - _v_cmd.y());
    _pEnd[i].z() = -STAND_HEIGHT; // 目标落脚高度
}

FSMStateName FSMState_Trot::checkTransition() {
    return FSMStateName::TROT;
}

void FSMState_Trot::exit() {
    std::cout << "[FSM] 退出 TROT 步态" << std::endl;
}
