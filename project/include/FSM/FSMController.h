/**********************************************************************
 * FSM 中央控制器 (FSMController)
 * 
 * 这个类是状态机的“指挥官”，负责：
 * 1. 管理所有具体的子状态对象 (Stand, Trot, Passive 等)。
 * 2. 调度当前状态的 Run() 逻辑。
 * 3. 监控状态切换请求，并执行平滑的切换流程。
***********************************************************************/
#ifndef FSMCONTROLLER_H
#define FSMCONTROLLER_H

#include "FSM/FSMState.h"
#include "FSM/FSMState_Passive.h" // 包含具体的被动状态
#include "FSM/ControlFSMData.h"
#include <map>

struct ControlFSMData; // 前向声明

class FSMController {
public:
    /**
     * @brief 构造函数
     * @param data 共享数据指针
     */
    FSMController(ControlFSMData* data);
    ~FSMController();

    /**
     * @brief 状态机的主循环入口 (通常在 main.cpp 的 500Hz 循环中调用)
     */
    void run();

private:
    /**
     * @brief 执行实际的状态切换逻辑
     * @param targetState 目标状态枚举名
     */
    void handleTransition(FSMStateName targetState);

    /**
     * @brief 安全检查：确保状态机始终处于有效状态
     */
    void safetyCheck();

    // 资源
    ControlFSMData* _data;
    
    // 状态容器：通过枚举名索引具体的状态对象
    std::map<FSMStateName, FSMState*> _statesList;

    // 当前正在运行的状态
    FSMState* _currentState;
    FSMStateName _nextStateName;
    FSMMode _mode;
};

#endif // FSMCONTROLLER_H
