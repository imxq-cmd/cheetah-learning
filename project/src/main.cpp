#include <iostream>
#include <unistd.h>
#include <csignal>
#include <cmath>
#include <iomanip>
#include <thread>
#include <chrono>

#include "common/robotConfig.h"
#include "interface/IOSDK.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

bool g_running = true;

// 信号处理句柄
void signalHandler(int signum) {
    g_running = false;
    std::cout << "\n[系统] 捕获信号 " << signum << "，准备退出..." << std::endl;
}

int main(int argc, char** argv) {
    // 注册 Ctrl+C 信号
    signal(SIGINT, signalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "   Cheetah Robot 硬件通信测试 (Hardware Test)" << std::endl;
    std::cout << "   功能: 验证串口收发 & 读取电机状态" << std::endl;
    std::cout << "   注意: 电机将处于被动阻尼模式 (KP=0, KD=1)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. 初始化硬件接口
    // 请确认 USB 端口名称正确
    IOSDK* ioInterface = new IOSDK("/dev/ttyUSB0", "/dev/ttyUSB1");

    // 2. 数据容器
    LowlevelCmd cmd;
    cmd.setZero(); // 初始清零
    
    // 设置轻微阻尼以防电机松动
    for(int i=0; i<12; i++){
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].Kd = 1.0; 
    }

    LowlevelState state;

    std::cout << "[系统] 开始主循环 (按 Ctrl+C 退出)..." << std::endl;
    
    // 计数器
    long long count = 0;
    auto next_wakeup = std::chrono::steady_clock::now();
    const std::chrono::microseconds interval_us(2000); // 500Hz

    while(g_running) {
        // --- 发送 & 接收 ---
        ioInterface->sendRecv(&cmd, &state);

        // --- 打印状态 (每 0.5 秒打印一次，避免刷屏) ---
        if (count % 250 == 0) {
            std::cout << "Count: " << count << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;
            std::cout << "|  ID  |    Pos (rad)   |    Vel (rad/s) |   Torque (Nm)  |" << std::endl;
            std::cout << "---------------------------------------------------------" << std::endl;
            
            // 打印前 3 个关节的数据作为示例
            // FR Leg (0, 1, 2)
            for(int i=0; i<3; i++) {
                std::cout << " |  " << std::setw(2) << i 
                          << " | " << std::setw(12) << state.motorState[i].q 
                          << " | " << std::setw(12) << state.motorState[i].dq
                          << " | " << std::setw(12) << state.motorState[i].tauEst 
                          << " |" << std::endl;
            }
            std::cout << "---------------------------------------------------------" << std::endl;
        }

        // --- 延时控制频率 (500Hz) ---
        next_wakeup += interval_us;
        std::this_thread::sleep_until(next_wakeup);
        count++;
    }

    delete ioInterface;
    return 0;
}

