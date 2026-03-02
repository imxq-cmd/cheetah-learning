/**********************************************************************
 * 通信接口抽象基类 (神经中枢入口)
 * 
 * 这是一个“抽象类”，定义了所有通信方式（如实机 SDK、仿真 ROS）必须具备的功能。
***********************************************************************/
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

class IOInterface {
public:
    IOInterface() {}
    virtual ~IOInterface() {}

    /**
     * @brief 发送指令并接收状态 (这是整个适配层的核心函数)
     * 
     * @param cmd   [输入] 来自算法的 12 关节统一指令
     * @param state [输出] 写入从硬件读取到的 12 关节统一状态
     */
    virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;

protected:
    // 这里未来可以添加类似于 CmdPanel 的用户输入接口
};

#endif // IOINTERFACE_H
