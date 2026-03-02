#ifndef FSMSTATE_TROT_H
#define FSMSTATE_TROT_H

#include "FSM/FSMState.h"
#include "Gait/WaveGenerator.h"
#include "Gait/FootTrajectory.h"

/**
 * Trot 步态状态机 (FSMState_Trot)
 * 核心逻辑：
 * 1. 使用 WaveGenerator 管理对角相位 (FR/RL 同步, FL/RR 同步)
 * 2. 使用 Raibert Heuristic 计算下一步落脚点 P_end
 * 3. 使用 FootTrajectory 生成摆动腿的摆线轨迹
 */
class FSMState_Trot : public FSMState {
public:
    FSMState_Trot(ControlFSMData* data);
    ~FSMState_Trot();

    virtual void enter() override;   // 进入时初始化步态时钟和足端初始位置
    virtual void run() override;     // 核心循环：计算进度、轨迹和 IK
    virtual void exit() override;    // 退出处理
    virtual FSMStateName checkTransition() override;

private:
    // 步态相关组件
    WaveGenerator*  _waveGait;
    FootTrajectory _footTrajectory[4];

    // 控制参数
    Vec3 _v_cmd;      // 期望速度 (x, y, yaw_rate)
    Vec3 _v_actual;   // 估算速度 (暂时假设 v_actual = v_cmd)

    // 状态记录
    Vec3 _pStart[4];  // 摆动开始时的起始足端位置
    Vec3 _pEnd[4];    // 预期的落脚位置

    // 内部函数
    void _calculateRaibertLP(int legID); // 计算 Raibert 落脚点
};

#endif // FSMSTATE_TROT_H
