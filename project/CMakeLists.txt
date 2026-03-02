cmake_minimum_required(VERSION 3.14)
project(cheetah_project)

# C++ Standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 项目配置
# 仅支持自研机器人 Cheetah
set(ROBOT_TYPE Cheetah)
set(PLATFORM amd64)         # 平台: amd64, arm64

# 编译配置
set(DEBUG OFF)              # 启用/禁用调试输出

# 应用配置
add_definitions(-DROBOT_TYPE_CHEETAH)
message(STATUS "[机器人配置] 自研四足机器人 - Cheetah")

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# ============================================================
# 第一阶段：Eigen矩阵库集成
# ============================================================
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ============================================================
# 第二阶段：电机SDK基础框架移植
# ============================================================
include_directories(${PROJECT_SOURCE_DIR}/include/unitree_actuator_sdk)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(MOTOR_SDK_LIB ${PROJECT_SOURCE_DIR}/lib/unitree_actuator_sdk/libUnitreeMotorSDK_Arm64.so)
else()
    set(MOTOR_SDK_LIB ${PROJECT_SOURCE_DIR}/lib/unitree_actuator_sdk/libUnitreeMotorSDK_Linux64.so)
endif()

# ============================================================
# 构建目标
# ============================================================
set(SOURCES
    src/main.cpp
    src/interface/IOSDK.cpp
    src/control/LegController.cpp
    src/FSM/FSMController.cpp
    src/FSM/FSMState.cpp
    src/FSM/FSMState_Passive.cpp
    src/FSM/FSMState_FixedStand.cpp
    src/FSM/FSMState_Trot.cpp
    src/Gait/WaveGenerator.cpp
    src/Gait/FootTrajectory.cpp
)

include_directories(${PROJECT_SOURCE_DIR}/include)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接库 (需要链接电机 SDK 的 SO 文件)
target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
    ${MOTOR_SDK_LIB}
    pthread
)

# 输出到 bin 目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
)

message(STATUS "")
message(STATUS "============================================================")
message(STATUS "项目 ${PROJECT_NAME} 配置成功")
message(STATUS "============================================================")
message(STATUS "  机器人类型: ${ROBOT_TYPE}")
message(STATUS "  计算平台: ${PLATFORM}")
message(STATUS "  调试模式: ${DEBUG}")
message(STATUS "  Eigen3位置: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "============================================================")
message(STATUS "")
