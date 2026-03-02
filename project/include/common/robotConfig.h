/**********************************************************************
 * 机器人配置定义
 * 
 * 仅保留自研四足机器人 Cheetah 的参数配置
***********************************************************************/
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "mathTypes.h"
#include <string>

// ============================================================
// Cheetah 机器人参数定义
// ============================================================

#define ROBOT_NAME "Cheetah"
#define ROBOT_DOF 12              // 12个自由度 (4条腿 x 3自由度)
#define ROBOT_LEGS 4              // 4条腿
#define LEG_DOF 3                 // 每条腿3个自由度
#define CONTROL_FREQ 500          // 控制频率 (Hz)

// 质量参数 (单位: kg)
#define ROBOT_MASS 12.0           // 机器人总质量
#define LEG_MASS 2.3              // 单条腿质量

// 几何参数 (单位: m)
#define LEG_LENGTH 0.3            // 腿长
#define ABAD_LINK_LENGTH 0.08     // 髋关节外展连杆长度 (L1)
#define HIP_LINK_LENGTH 0.2       // 大腿连杆长度 (L2)
#define KNEE_LINK_LENGTH 0.2      // 小腿连杆长度 (L3)
#define HIP_TO_HIP_X 0.2          // 髋关节间距X方向
#define HIP_TO_HIP_Y 0.15         // 髋关节间距Y方向

// 运行姿态控制参数 (单位: m)
#define STAND_HEIGHT 0.33         // 目标站立高度 (相对该腿髋关节的 Z 距离)
#define STAND_UP_TIME 2.0         // 起立动作持续时间 (秒)
#define SWING_HEIGHT 0.1          // 默认摆动抬腿高度 (秒)
#define GRC_TROT_DURATION 0.5     // Trot 步态周期 (秒)

namespace Cheetah {
    static constexpr double abadLinkLength = 0.08;
    static constexpr double hipLinkLength = 0.2;
    static constexpr double kneeLinkLength = 0.2;
    
    // 默认足端位置 (相对于该腿髋关节坐标系)
    static const Vec3 defaultFootPos(0.0, 0.0, -0.25); 

    // --- 关节零偏 (Joint Offset) ---
    static const Vec3 jointOffset_1(  20.52 * M_PI / 180.0, 
                                      57.18 * M_PI / 180.0, 
                                    -147.8  * M_PI / 180.0 );

    static const Vec3 jointOffset_2( -20.52 * M_PI / 180.0, 
                                     -57.18 * M_PI / 180.0, 
                                      147.8  * M_PI / 180.0 );

    static const Vec3 jointOffset_3(  20.52 * M_PI / 180.0, 
                                     -57.18 * M_PI / 180.0, 
                                      147.8  * M_PI / 180.0 );

    static const Vec3 jointOffset_4( -20.52 * M_PI / 180.0, 
                                      57.18 * M_PI / 180.0, 
                                    -147.8  * M_PI / 180.0 );

    // --- 关节传动方向 (Joint Direction) ---
    static const Vec3 jointDir_1( 1.0, -1.0, -1.0); // FR (Abad, Hip, Knee)
    static const Vec3 jointDir_2( 1.0,  1.0,  1.0); // FL (Abad, Hip, Knee)
    static const Vec3 jointDir_3(-1.0,  1.0,  1.0); // RL (Abad, Hip, Knee)
    static const Vec3 jointDir_4(-1.0, -1.0, -1.0); // RR (Abad, Hip, Knee)
}

// 关节限制 (单位: rad)
#define LEG_JOINT_MAX 1.57        // 关节最大角度 (π/2)
#define LEG_JOINT_MIN -1.57       // 关节最小角度 (-π/2)
#define LEG_VELOCITY_MAX 10.0     // 最大速度 (rad/s)
#define LEG_TORQUE_MAX 30.0       // 最大力矩 (Nm)

// 机器人结构类型
#define ROBOT_STRUCTURE_TYPE "四足步行机器人"
#define ROBOT_LEG_TYPE "串联结构"

// 参考点坐标 - 腿部相对于躯体中心的位置 (FR, FL, RR, RL)
static const Vec34 LEG_OFFSETS = (Vec34() << 
     0.1,  -0.1,  -0.1,   0.1,     // X 坐标
     0.08,  0.08, -0.08, -0.08,    // Y 坐标
     0.0,   0.0,   0.0,   0.0      // Z 坐标
).finished();

// ============================================================
// 通用工具函数
// ============================================================

/**
 * 获取当前机器人的名称
 */
inline std::string getRobotName()
{
    return std::string(ROBOT_NAME);
}

/**
 * 获取腿部参数
 */
struct LegParameters
{
    int dof;              // 自由度数
    double max_velocity;  // 最大速度
    double max_torque;    // 最大力矩
    double max_angle;     // 最大关节角度
    double min_angle;     // 最小关节角度
};

inline LegParameters getLegParameters()
{
    LegParameters params;
    params.dof = LEG_DOF;
    params.max_velocity = LEG_VELOCITY_MAX;
    params.max_torque = LEG_TORQUE_MAX;
    params.max_angle = LEG_JOINT_MAX;
    params.min_angle = LEG_JOINT_MIN;
    return params;
}

#endif  // ROBOT_CONFIG_H
