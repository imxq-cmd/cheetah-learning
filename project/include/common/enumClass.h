/**********************************************************************
 * FSM 状态机核心枚举定义
 * 
 * 这里的定义决定了机器人有哪些控制模式，以及每个状态内部的运行阶段。
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

/**
 * @brief FSM 状态枚举 (决定机器人当前在做什么)
 */
enum class FSMStateName {
    INVALID,
    PASSIVE,        // 被动模式 (全掉电，最安全)
    FIXEDSTAND,     // 缓慢起立 (轨迹插值)
    FREESTAND,      // 平衡站立 (支持重心控制)
    TROT,           // 对角小跑 (核心步态)
    SWINGTEST,      // 腿部摆动测试 (调试用)
    RECOVERYSTAND,  // 跌倒恢复 (翻身)
};

/**
 * @brief FSM 运行阶段 (描述状态内部的微观生命周期)
 */
enum class FSMMode {
    NORMAL,         // 正常执行 Run()
    TRANSITIONING,  // 正在加载或平滑过渡
    EXIT            // 正在退出清理
};

#endif // ENUMCLASS_H
