/**********************************************************************
 * 电机 SDK 适配类 (实机控制的具体实现)
 * 
 * 这个类负责：
 * 1. 管理具体的串口通信 (SerialPort)
 * 2. 循环遍历 12 个关节，完成 [统一格式 <-> 电机 SDK 格式] 的转换
***********************************************************************/
#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include "unitree_actuator_sdk/unitreeMotor/unitreeMotor.h"
#include "unitree_actuator_sdk/serialPort/SerialPort.h"
#include <vector>

class IOSDK : public IOInterface {
public:
    /**
     * @param devName1 串口1设备名称 (前腿), 例如 "/dev/ttyUSB1"
     * @param devName2 串口2设备名称 (后腿), 例如 "/dev/ttyUSB2"
     */
    IOSDK(const char* devName1, const char* devName2);
    ~IOSDK();

    /**
     * 实现基类的 sendRecv 函数
     */
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) override;

private:
    SerialPort _serial1;          // 串口对象1 (前腿)
    SerialPort _serial2;          // 串口对象2 (后腿)
    MotorCmd   _motorCmd[12];     // SDK 原始指令包数组
    MotorData  _motorData[12];    // SDK 原始状态包数组

    Vec3 _offsets[4];            // 存储4条腿的关节偏置向量
    Vec3 _directions[4];         // 存储4条腿的关节方向向量 (1.0 或 -1.0)
    void initMotorSettings();    // 内部初始化函数
};

#endif // IOSDK_H
