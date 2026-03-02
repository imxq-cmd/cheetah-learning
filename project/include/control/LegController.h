/**********************************************************************
 * 腿部控制器 (LegController)
 * 
 * 这个类负责：
 * 1. 腿部运动学计算 (正解 FK / 逆解 IK)
 * 2. 状态更新 (将关节反馈转换为足端位置和速度)
 * 3. 动力学控制 (将足端力转换为关节力矩)
***********************************************************************/
#ifndef LEGCONTROLLER_H
#define LEGCONTROLLER_H

#include "common/mathTypes.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

/**
 * @brief 单腿数据结构 (存储该腿的所有运动学信息)
 */
struct LegData {
    Vec3 q;       // 关节角度 [abad, hip, knee]
    Vec3 dq;      // 关节角速度
    Vec3 tau;     // 实际关节力矩
    Vec3 p;       // 足端位置 (相对于该腿髋关节坐标系)
    Vec3 v;       // 足端速度 (相对于该腿髋关节坐标系)
    Mat3 J;       // 雅可比矩阵 (Jacobian Matrix)

    LegData() {
        q.setZero(); dq.setZero(); tau.setZero();
        p.setZero(); v.setZero(); J.setZero();
    }
};

/**
 * @brief 单腿控制指令结构
 */
struct LegCommand {
    Vec3 q;       // 期望关节角度
    Vec3 dq;      // 期望关节角速度
    Vec3 tau;     // 前馈力矩
    float kp;     // 位置增益
    float kd;     // 速度增益
    
    Vec3 pDes;    // 期望足端位置 (仅用于记录/调试)
    Vec3 vDes;    // 期望足端速度

    LegCommand() {
        q.setZero(); dq.setZero(); tau.setZero();
        kp = 0; kd = 0;
        pDes.setZero(); vDes.setZero();
    }
};

class LegController {
public:
    LegController();
    ~LegController();

    /**
     * @brief 根据底层反馈状态更新腿部运动学数据
     * @param state 底层反馈的状态包
     */
    void updateState(const LowlevelState* state);

    /**
     * @brief 将计算好的控制指令写入底层发送包
     * @param cmd 底层发送指令包
     */
    void updateCommand(LowlevelCmd* cmd);

    /**
     * @brief 简单的运动学正解 (Forward Kinematics)
     * @param legID 腿索引 0-3
     * @param q 关节角度向量
     * @return Vec3 足端位置 (相对于该腿髋关节)
     */
    Vec3 getFK(int legID, Vec3 q);

    /**
     * @brief 简单的运动学逆解 (Inverse Kinematics)
     * @param legID 腿索引 0-3
     * @param p 足端目标位置
     * @param q 返回的关节角度向量
     */
    bool getIK(int legID, Vec3 p, Vec3& q);

    /**
     * @brief 计算解析式雅可比矩阵 (Jacobian Matrix)
     * @param legID 腿索引 0-3
     * @param q 当前关节角
     * @return Mat3 3x3 雅可比矩阵
     */
    Mat3 getJacobian(int legID, Vec3 q);

    // 存储 4 条腿的数据
    LegData data[4];
    LegCommand commands[4]; // 存储 4 条腿的控制指令

    // 兼容旧接口别名 (如果代码中有用到 computeFK/IK 的话)
    // 建议统一使用 getFK / getIK
    Vec3 computeFK(int legID, Vec3 q) { return getFK(legID, q); }
    bool computeIK(int legID, Vec3 p, Vec3& q) { return getIK(legID, p, q); }

private:
    // 物理参数 (使用 abad/hip/knee 命名)
    double _l1, _l2, _l3;
};

#endif // LEGCONTROLLER_H
