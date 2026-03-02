#ifndef FOOTTRAJECTORY_H
#define FOOTTRAJECTORY_H

#include "common/mathTypes.h"

/**
 * FootTrajectory: 足端摆动轨迹生成器 (基于摆线 Cycloid)
 * 负责根据摆动进度计算 3D 空间中的期望足端位置
 */
class FootTrajectory {
public:
    FootTrajectory();
    ~FootTrajectory();

    /**
     * 设置轨迹的起点和终点
     * @param pStart 摆动开始时的足端位置 (相对于该腿髋关节)
     * @param pEnd   预期落脚位置 (相对于该腿髋关节)
     */
    void setFootPos(Vec3 pStart, Vec3 pEnd);

    /**
     * 计算当前进度的足端位置
     * @param phi 摆动相进度 [0, 1]
     * @param swingHeight 抬腿高度 (米)
     * @return 当前时刻的期望足端位置 (Vec3)
     */
    Vec3 getFootPos(float phi, float swingHeight);

private:
    Vec3 _pStart; // 起点
    Vec3 _pEnd;   // 终点
    Vec3 _pCur;   // 当前点
};

#endif // FOOTTRAJECTORY_H
