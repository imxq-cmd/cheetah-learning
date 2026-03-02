#ifndef __UNITREEMOTOR_H
#define __UNITREEMOTOR_H

#include "unitreeMotor/include/motor_msg_GO-M8010-6.h" 
#include <stdint.h>
#include <iostream>

enum class MotorType{
    GO_M8010_6
};

enum class MotorMode{
    BRAKE, 
    FOC, 
    CALIBRATE
};

/**
 * @brief 电机控制指令结构体（仅支持 GO_M8010_6）
 */
struct MotorCmd{
    public:
        MotorCmd(){}
        MotorType motorType = MotorType::GO_M8010_6;
    int hex_len;                   
        unsigned short id;              
        unsigned short mode;           
        float tau;       // 期望力矩 (Nm)                 
        float dq;        // 期望速度 (rad/s)               
        float q;         // 期望位置 (rad)            
        float kp;        // 比例增益             
        float kd;        // 微分增益             

        void modify_data(MotorCmd* motor_s); 
        uint8_t* get_motor_send_data();

    private:
        ControlData_t  GO_M8010_6_motor_send_data;  
};

/**
 * @brief 电机反馈数据结构体（仅支持 GO_M8010_6）
 */
struct MotorData{
    public:
        MotorData(){}
        MotorType motorType = MotorType::GO_M8010_6;
        int hex_len;                   
        unsigned char motor_id;         
        unsigned char mode;             
        int temp;                      
        int merror;          
        float tau;       // 实际力矩 (Nm)                
        float dq;        // 实际速度 (rad/s)               
        float q;         // 实际位置 (rad)            
       
        bool correct = false;                  
        bool extract_data(MotorData* motor_r);
        uint8_t* get_motor_recv_data();
       
        int footForce;   // 足端压力
        float LW;        // 低速反馈                      
        int Acc;         // 加速度                     

        float gyro[3];   // 陀螺仪数据               
        float acc[3];    // 加速度计数据
    private:
        MotorData_t GO_M8010_6_motor_recv_data;    
};

// 工具函数
int queryMotorMode(MotorType motortype, MotorMode motormode);
float queryGearRatio(MotorType motortype);

#endif  // __UNITREEMOTOR_H
