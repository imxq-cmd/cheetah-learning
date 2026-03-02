#include "Gait/FootTrajectory.h"
#include <cmath>

FootTrajectory::FootTrajectory() {
    _pStart.setZero();
    _pEnd.setZero();
    _pCur.setZero();
}

FootTrajectory::~FootTrajectory() {}

void FootTrajectory::setFootPos(Vec3 pStart, Vec3 pEnd) {
    _pStart = pStart;
    _pEnd = pEnd;
}

Vec3 FootTrajectory::getFootPos(float phi, float swingHeight) {
    _pCur = _pStart;

    // 1. 水平方向 (X, Y) 采用线性插值 (线性推进)
    _pCur(0) = _pStart(0) + (_pEnd(0) - _pStart(0)) * phi;
    _pCur(1) = _pStart(1) + (_pEnd(1) - _pStart(1)) * phi;

    // 2. 竖直方向 (Z) 采用摆线 (Cycloid) 插值
    // 摆线方程分两段：0-0.5(上升) 和 0.5-1.0(下降)
    // 这种实现能确保在最高点 (phi=0.5) 达到 swingHeight，且起止点高度平滑过渡
    float z_offset = 0;
    
    if (phi < 0.5f) {
        // 上半段: 从起点高度上升到最高点 (起点高度 + swingHeight)
        float normalized_phi = phi / 0.5f;
        z_offset = swingHeight * (1.0f / M_PI * (2.0f * M_PI * normalized_phi - std::sin(2.0f * M_PI * normalized_phi)));
        _pCur(2) = _pStart(2) + z_offset;
    } else {
        // 下半段: 从最高点下降到终点高度 (起点高度 + (终点-起点)高度)
        float normalized_phi = (phi - 0.5f) / 0.5f;
        z_offset = swingHeight * (1.0f - (1.0f / M_PI * (2.0f * M_PI * normalized_phi - std::sin(2.0f * M_PI * normalized_phi))));
        // 这里的逻辑考虑了起点和终点的高度差（如上台阶）
        _pCur(2) = _pStart(2) + (_pEnd(2) - _pStart(2)) * normalized_phi + z_offset;
    }

    return _pCur;
}
