#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"
#include <algorithm>

/**
 * WaveGenerator: 步态周期波形发生器
 * 负责管理 4 条腿的状态切换 (支撑/摆动) 和相位进度
 */
class WaveGenerator {
public:
    /**
     * @param T 步态总周期 (秒)
     * @param duty 支撑相占比 (通常为 0.5)
     * @param phase 暂不使用的参数，逻辑中已硬编码对角相位
     */
    WaveGenerator(double T, double duty, Vec4 phase = Vec4::Zero());
    ~WaveGenerator();

    void update(double dt);  // 每步调用，更新相位
    void restart();          // 重置时钟/相位

    // 输出接口
    float getStatus(int legID);      // 获取该腿状态: 1(支撑), 0(摆动)
    float getPhase(int legID);       // 获取该腿在当前属于该相位的进度 [0, 1]
    float getPhaseInFullCycle(int legID); // 获取该腿在整个大周期中的位置

private:
    double _T;        // 总周期
    double _duty;     // 支撑占比
    Vec4   _offset;   // 腿间相位偏移
    
    Vec4   _phase;    // 当前整周期进度 [0, 1]
    int    _status[4]; // 支撑/摆动标志
    float  _phaseInPhase[4]; // 在当前相(支撑/摆动)内的内部进度 [0, 1]
};

#endif // WAVEGENERATOR_H
