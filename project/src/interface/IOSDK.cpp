#include "interface/IOSDK.h"
#include "common/robotConfig.h"
#include <iostream>

IOSDK::IOSDK(const char* devName1, const char* devName2) 
    : _serial1(devName1), _serial2(devName2) {
    
    // 初始化 4 条腿各自独立的偏置向量 (索引 0-3 分别对应 1-4 号腿)
    _offsets[0] = Cheetah::jointOffset_1; // FR
    _offsets[1] = Cheetah::jointOffset_2; // FL
    _offsets[2] = Cheetah::jointOffset_3; // RL
    _offsets[3] = Cheetah::jointOffset_4; // RR

    // 初始化旋转方向
    _directions[0] = Cheetah::jointDir_1;
    _directions[1] = Cheetah::jointDir_2;
    _directions[2] = Cheetah::jointDir_3;
    _directions[3] = Cheetah::jointDir_4;

    std::cout << "[通信层] 初始化双串口并加载 4 腿独立关节偏置与方向" << std::endl;
    initMotorSettings();
}

IOSDK::~IOSDK() {
}

void IOSDK::initMotorSettings() {
    // 为 12 个电机分配 ID 和类型 (Cheetah 架构)
    // 当前映射定义 (用户自定义):
    // 0-2 (FR-右前), 3-5 (FL-左前), 6-8 (RL-左后), 9-11 (RR-右后)
    for(int i=0; i<12; ++i) {
        _motorCmd[i].motorType = MotorType::GO_M8010_6;
        _motorData[i].motorType = MotorType::GO_M8010_6;
        _motorCmd[i].id = i + 1; // 数组索引 0-11 对应物理电机 ID 1-12
        
        // 初始模式设置为 FOC 闭环
        _motorCmd[i].mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    }
}

/**
 * 适配层的核心逻辑：
 * 1. 将 LowlevelCmd 的内容翻译给 12 个 _motorCmd
 * 2. 通过串口一个个发送、接收数据
 * 3. 将得到的 _motorData 翻译回 LowlevelState
 */
void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state) {
    
    for(int i=0; i<12; ++i) {
        int legID = i / 3;
        int jointID = i % 3;

        // 物理角度 = (算法角度 * 方向系数) + 偏移量
        _motorCmd[i].q = (float)(cmd->motorCmd[i].q * _directions[legID](jointID) + _offsets[legID](jointID));
        _motorCmd[i].dq = (float)(cmd->motorCmd[i].dq * _directions[legID](jointID));
        _motorCmd[i].tau = (float)(cmd->motorCmd[i].tau * _directions[legID](jointID));
        _motorCmd[i].kp = (float)cmd->motorCmd[i].Kp;
        _motorCmd[i].kd = (float)cmd->motorCmd[i].Kd;

        // 根据索引选择串口
        if(i < 6) {
            _serial1.sendRecv(&_motorCmd[i], &_motorData[i]);
        } else {
            _serial2.sendRecv(&_motorCmd[i], &_motorData[i]);
        }

        // 算法角度 = (物理反馈 - 偏移量) / 方向系数
        if(_motorData[i].correct) {
            state->motorState[i].q = (_motorData[i].q - _offsets[legID](jointID)) / _directions[legID](jointID);
            state->motorState[i].dq = _motorData[i].dq / _directions[legID](jointID);
            state->motorState[i].tau = _motorData[i].tau / _directions[legID](jointID);
        }
    }
}
