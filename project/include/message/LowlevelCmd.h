/**********************************************************************
 * 底层控制指令数据结构 (统一接口)
 * 
 * 这个文件定义了高层控制算法如何“下达命令”给所有关节。
 * 它使用 Eigen 向量来存储 12 个关节的物理量，方便矩阵运算。
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"

struct MotorCmd; // 前向声明输出给 SDK 的结构体

/**
 * @brief 单个关节的控制参数
 */
struct JointCmd {
    double q;    // 目标位置 (rad)
    double dq;   // 目标速度 (rad/s)
    double tau;  // 目标力矩 (Nm)
    double Kp;   // 比例增益 (位置刚度)
    double Kd;   // 微分增益 (速度阻尼)

    JointCmd() {
        q = 0.0; dq = 0.0; tau = 0.0; Kp = 0.0; Kd = 0.0;
    }
};

/**
 * @brief 机器人底层的整体控制指令包 (12个关节)
 */
struct LowlevelCmd {
    // 关节映射定义:
    // 0-2: FR (右前)
    // 3-5: FL (左前)
    // 6-8: RL (左后)
    // 9-11: RR (右后)
    JointCmd motorCmd[12];

    // 提供一些工具函数，方便直接用 Eigen 向量赋值
    void setQ(const Vec12& qVec) {
        for(int i=0; i<12; ++i) motorCmd[i].q = qVec(i);
    }
    
    void setTau(const Vec12& tauVec) {
        for(int i=0; i<12; ++i) motorCmd[i].tau = tauVec(i);
    }
    
    // 清零所有指令
    void setZero() {
        for(int i=0; i<12; ++i) {
            motorCmd[i].q = 0; motorCmd[i].dq = 0; motorCmd[i].tau = 0;
            motorCmd[i].Kp = 0; motorCmd[i].Kd = 0;
        }
    }
};

#endif // LOWLEVELCMD_H
