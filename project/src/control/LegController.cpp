#include "control/LegController.h"
#include "common/robotConfig.h"
#include <cmath>
#include <iostream>

LegController::LegController() {
    // 初始化物理参数 (使用习惯命名: abad, hip, knee)
    _l1 = Cheetah::abadLinkLength;
    _l2 = Cheetah::hipLinkLength;
    _l3 = Cheetah::kneeLinkLength;
}

LegController::~LegController() {}

void LegController::updateState(const LowlevelState* state) {
    for (int i = 0; i < 4; i++) {
        // 1. 提取该腿的 3 个关节角度 (0-2, 3-5, 6-8, 9-11)
        for (int j = 0; j < 3; j++) {
            data[i].q(j) = state->motorState[i * 3 + j].q;
            data[i].dq(j) = state->motorState[i * 3 + j].dq;
            data[i].tau(j) = state->motorState[i * 3 + j].tauEst;
        }

        // 2. 通过正解更新足端位置 (FK)
        data[i].p = getFK(i, data[i].q);

        // 3. 计算雅可比矩阵 (J)
        data[i].J = getJacobian(i, data[i].q);

        // 4. 计算足端速度 v = J * dq
        data[i].v = data[i].J * data[i].dq;
    }
}

void LegController::updateCommand(LowlevelCmd* cmd) {
    for(int i=0; i<12; i++) {
        int legID = i / 3;
        int jointID = i % 3;

        // 根据 commands[legID] 填充底层指令
        cmd->motorCmd[i].q = (float)commands[legID].q(jointID);
        cmd->motorCmd[i].dq = (float)commands[legID].dq(jointID);
        cmd->motorCmd[i].tau = (float)commands[legID].tau(jointID);
        cmd->motorCmd[i].Kp = (float)commands[legID].kp;
        cmd->motorCmd[i].Kd = (float)commands[legID].kd;
    }
}

Vec3 LegController::getFK(int legID, Vec3 q) {
    Vec3 p;

    double l1 = (legID == 1 || legID == 2) ? _l1 : -_l1; 
    
    double s1 = std::sin(q(0));
    double c1 = std::cos(q(0));
    double s2 = std::sin(q(1));
    double c2 = std::cos(q(1));
    double s23 = std::sin(q(1) + q(2));
    double c23 = std::cos(q(1) + q(2));

    // 正运动学解析解 (足端相对于该腿髋关节转动中心)
    // x: 向前为正
    p(0) = _l2 * s2 + _l3 * s23;
    // y: 向左为正
    p(1) = l1 * c1 + _l2 * s1 * c2 + _l3 * s1 * c23;
    // z: 向上为正 (通常足端在髋关节下方，所以 z 为负值)
    p(2) = l1 * s1 - _l2 * c1 * c2 - _l3 * c1 * c23;

    return p;
}

bool LegController::getIK(int legID, Vec3 p, Vec3& q) {
    // 逆运动学几何解
    // 左腿 (FL, RL) L1 为正; 右腿 (FR, RR) L1 为负
    double l1 = (legID == 1 || legID == 2) ? _l1 : -_l1;
    
    // 1. 计算 Abad 关节 (q0)
    double r_yz = std::sqrt(p(1) * p(1) + p(2) * p(2));
    if (r_yz < std::abs(l1)) return false; 
    
    q(0) = std::atan2(p(1), -p(2)) - std::acos(l1 / r_yz);

    // 2. 将位置投影到大腿/小腿转动平面
    double x = p(0);
    double y_plane = -std::sqrt(r_yz * r_yz - l1 * l1); 

    // 3. 计算 Hip (q1) 和 Knee (q2) - 余弦定理
    double dist_sq = x * x + y_plane * y_plane;
    double dist = std::sqrt(dist_sq);
    double D = (dist_sq - _l2 * _l2 - _l3 * _l3) / (2 * _l2 * _l3);
    if (std::abs(D) > 1.0) return false; 

    q(2) = std::acos(D); 
    
    double theta = std::atan2(y_plane, x);
    double phi = std::acos((_l2 * _l2 + dist_sq - _l3 * _l3) / (2 * _l2 * dist));
    
    q(1) = theta + phi;

    return true;
}

Mat3 LegController::getJacobian(int legID, Vec3 q) {
    Mat3 J = Mat3::Zero();

    double l1 = (legID == 1 || legID == 2) ? _l1 : -_l1; 
    
    double s1 = std::sin(q(0));
    double c1 = std::cos(q(0));
    double s2 = std::sin(q(1));
    double c2 = std::cos(q(1));
    double s23 = std::sin(q(1) + q(2));
    double c23 = std::cos(q(1) + q(2));

    // 计算中间项以简化表达式
    double l2c2_l3c23 = _l2 * c2 + _l3 * c23;
    double l2s2_l3s23 = _l2 * s2 + _l3 * s23;

    // 第一列: 对 q0 (Abad) 求偏导
    J(0, 0) = 0;
    J(1, 0) = -l1 * s1 + c1 * l2c2_l3c23;
    J(2, 0) =  l1 * c1 + s1 * l2c2_l3c23;

    // 第二列: 对 q1 (Hip) 求偏导
    J(0, 1) =  l2c2_l3c23;
    J(1, 1) = -s1 * l2s2_l3s23;
    J(2, 1) =  c1 * l2s2_l3s23;

    // 第三列: 对 q2 (Knee) 求偏导
    J(0, 2) = _l3 * c23;
    J(1, 2) = -_l3 * s1 * s23;
    J(2, 2) =  _l3 * c1 * s23;

    return J;
}
