#include "FSM/FSMController.h"
#include "FSM/FSMState_Passive.h"
#include "FSM/FSMState_FixedStand.h"
#include "FSM/FSMState_Trot.h"
#include <iostream>

FSMController::FSMController(ControlFSMData* data) : _data(data) {
    // 1. 初始化所有预设的状态对象
    _statesList[FSMStateName::PASSIVE] = new FSMState_Passive(data);
    _statesList[FSMStateName::FIXEDSTAND] = new FSMState_FixedStand(data); 
    _statesList[FSMStateName::TROT] = new FSMState_Trot(data);
    
    // 2. 设定初始状态为 PASSIVE (最安全)
    _currentState = nullptr; // 初始为空
    _nextStateName = FSMStateName::PASSIVE; // 下一帧将切换到 PASSIVE
    _mode = FSMMode::TRANSITIONING;

    std::cout << "[FSM] 控制器初始化完成" << std::endl;
}

FSMController::~FSMController() {
    // 释放所有状态对象的内存
    for (auto const& [name, state] : _statesList) {
        delete state;
    }
}

/**
 * 核心调度逻辑：
 * 这个函数每 2ms 运行一次。它就像一个看门人，不断确认：
 * 1. 我现在该干什么 (Run)
 * 2. 我现在该不该换个事情干 (CheckTransition)
 */
void FSMController::run() {
    
    // --- 1. 状态切换检查与处理 ---
    // 每个状态都有权利告诉指挥官：“我觉得该切换了”
    if (_mode == FSMMode::NORMAL) {
        FSMStateName targetState = _currentState->checkTransition();
        if (targetState != FSMStateName::INVALID) {
            handleTransition(targetState);
        }
    }

    // --- 2. 状态逻辑执行 ---
    // 根据当前模式决定是执行正常的 Run 还是执行切换流程
    if (_mode == FSMMode::NORMAL) {
        _currentState->run();
    } 
    else if (_mode == FSMMode::TRANSITIONING) {
        // [切换逻辑的核心：Enter 钩子]
        // 如果是刚从别的状态跳过来，先执行新状态的初始化
        if (_currentState != nullptr) _currentState->exit();
        
        _currentState = _statesList[_nextStateName];
        _currentState->enter();
        
        _mode = FSMMode::NORMAL; // 切换完成，进入正常循环
        std::cout << "[FSM] 状态已成功切换至: " << _currentState->stateNameStr() << std::endl;
    }

    // --- 3. 安全监控 ---
    safetyCheck();
}

void FSMController::handleTransition(FSMStateName targetState) {
    if (_statesList.count(targetState) > 0) {
        _nextStateName = targetState;
        _mode = FSMMode::TRANSITIONING;
    } else {
        std::cerr << "[FSM] 致命错误：尝试切换到未定义的状态！" << std::endl;
    }
}

void FSMController::safetyCheck() {
    // 这里可以后期添加：
    // 如果 IMU 检测到倾角过大，或者电机通信丢失，强制 handleTransition(FSMStateName::PASSIVE)
}
