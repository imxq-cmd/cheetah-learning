#ifndef FSMSTATE_FIXEDSTAND_H
#define FSMSTATE_FIXEDSTAND_H

#include "FSM/FSMState.h"
#include "common/mathTypes.h"

/**
 * 固定起立状态 (FSMState_FixedStand)
 * 目标：从当前关节角度平滑过渡到预设的站立高度
 */
class FSMState_FixedStand : public FSMState {
public:
    FSMState_FixedStand(ControlFSMData* data);
    
    virtual void enter() override;                  // 进入时记录当前角度、计算目标
    virtual void run() override;                    // 进循环插值发送
    virtual void exit() override;                   // 退出清理
    virtual FSMStateName checkTransition() override; // 检查是否完成过渡

private:
    float _duration = 2.0f;  // 起立时间 (秒)
    float _counter = 0;      // 内部计数器
    
    Vec3 _startJointAngles[4];  // 4条腿的起始角度
    Vec3 _targetJointAngles[4]; // 4条腿的目标角度
    
    // Kp, Kd 指令 (起立阶段建议偏小以保证安全)
    float _stiff_kp = 40.0f;
    float _stiff_kd = 2.0f;
};

#endif // FSMSTATE_FIXEDSTAND_H
