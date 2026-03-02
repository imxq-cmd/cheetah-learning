/**********************************************************************
 * 底层反馈状态数据结构 (统一接口)
 * 
 * 这个文件定义了所有关节的实时状态如何“反馈给”高层算法。
***********************************************************************/
#ifndef LOWLEVELSTATE_H
#define LOWLEVELSTATE_H

#include "common/mathTypes.h"

/**
 * @brief 单个关节的反馈状态
 */
struct JointState {
    double q;      // 实际位置 (rad)
    double dq;     // 实际速度 (rad/s)
    double tauEst; // 实际力矩 (Nm) - 估计值
    int temp;      // 电机温度 (°C)

    JointState() {
        q = 0.0; dq = 0.0; tauEst = 0.0; temp = 0;
    }
};

/**
 * @brief 机器人底层的整体状态包
 */
struct LowlevelState {
    // 关节映射定义 (用户自定义):
    // 0-2 (FR-右前), 3-5 (FL-左前), 6-8 (RL-左后), 9-11 (RR-右后)
    JointState motorState[12];

    // IMU 状态 (机身姿态)
    Vec3 imu_acc;    // 加速度计
    Vec3 imu_gyro;   // 陀螺仪
    Quat imu_quat;   // 四元数姿态
    
    // 足端力反馈 (可选)
    VecInt4 footForce;

    // 转换为 Eigen 向量，方便算法使用
    Vec12 getQ() {
        Vec12 qVec;
        for(int i=0; i<12; ++i) qVec(i) = motorState[i].q;
        return qVec;
    }

    Vec12 getDq() {
        Vec12 dqVec;
        for(int i=0; i<12; ++i) dqVec(i) = motorState[i].dq;
        return dqVec;
    }
};

#endif // LOWLEVELSTATE_H
