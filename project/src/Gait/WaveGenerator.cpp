#include "Gait/WaveGenerator.h"

WaveGenerator::WaveGenerator(double T, double duty, Vec4 phase) 
    : _T(T), _duty(duty) {
    // 强制设定为 Trot 步态的对角相位关系:
    // ID 0:FR, ID 1:FL, ID 2:RL, ID 3:RR
    // 组 A (相位 0.0): FR (0) 和 RL (2)
    // 组 B (相位 0.5): FL (1) 和 RR (3)
    _offset << 0.0, 0.5, 0.0, 0.5;
    
    _phase.setZero();
    for(int i=0; i<4; i++) {
        _status[i] = 0;
        _phaseInPhase[i] = 0.0f;
    }
}

WaveGenerator::~WaveGenerator() {}

void WaveGenerator::update(double dt) {
    for(int i=0; i<4; i++) {
        // 1. 计算该腿在整个大周期中的总进度 [0, 1]
        _phase(i) += dt / _T;
        if(_phase(i) > 1.0) _phase(i) -= 1.0;

        // 根据初始相位偏移调整
        double actualPhase = _phase(i) + _offset(i);
        if(actualPhase > 1.0) actualPhase -= 1.0;

        // 2. 根据支撑占比划分状态 (以 0.5 为例)
        if(actualPhase < _duty) {
            _status[i] = 1; // 支撑相 (Stance)
            _phaseInPhase[i] = actualPhase / _duty;
        } else {
            _status[i] = 0; // 摆动相 (Swing)
            _phaseInPhase[i] = (actualPhase - _duty) / (1.0 - _duty);
        }
    }
}

void WaveGenerator::restart() {
    _phase.setZero();
    for(int i=0; i<4; i++) {
        _status[i] = 0;
        _phaseInPhase[i] = 0.0f;
    }
}

float WaveGenerator::getStatus(int legID) { return (float)_status[legID]; }
float WaveGenerator::getPhase(int legID) { return _phaseInPhase[legID]; }
float WaveGenerator::getPhaseInFullCycle(int legID) { return (float)_phase(legID); }
