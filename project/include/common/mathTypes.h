#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <cmath>
#include <iostream>
#include <algorithm>
#include <eigen3/Eigen/Dense>

/**
 * 线性插值工具函数 (Linear Interpolation)
 * @param ini 起始值
 * @param target 目标值
 * @param fraction 进度系数 [0, 1]
 */
template <typename T1, typename T2>
inline T1 jointLinearInterp(const T1 &ini, const T1 &target, T2 fraction) {
  T2 weight = std::max(T2(0), std::min(fraction, T2(1)));
  return ini * (T2(1) - weight) + target * weight;
}

/************************/
/****** 一维向量 *****/
/************************/
// 2x1 向量
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 3x1 向量
using Vec3 = typename Eigen::Matrix<double, 3, 1>;

// 4x1 向量
using Vec4 = typename Eigen::Matrix<double, 4, 1>;

// 6x1 向量
using Vec6 = typename Eigen::Matrix<double, 6, 1>;

// 四元数 (4x1)
using Quat = typename Eigen::Matrix<double, 4, 1>;

// 4x1 整数向量
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

// 12x1 向量 (4条腿，每条腿3个自由度)
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

// 18x1 向量
using Vec18 = typename Eigen::Matrix<double, 18, 1>;

// 动态长度向量
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/****** 二维矩阵 *****/
/************************/
// 旋转矩阵 (3x3)
using RotMat = typename Eigen::Matrix<double, 3, 3>;

// 齐次变换矩阵 (4x4)
using HomoMat = typename Eigen::Matrix<double, 4, 4>;

// 2x2 矩阵
using Mat2 = typename Eigen::Matrix<double, 2, 2>;

// 3x3 矩阵
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

// 3x4 矩阵 (每列是一个3x1向量)
using Vec34 = typename Eigen::Matrix<double, 3, 4>;

// 6x6 矩阵
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

// 12x12 矩阵
using Mat12 = typename Eigen::Matrix<double, 12, 12>;

// 动态尺寸矩阵
using MatX = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/************************/
/*** 单位矩阵定义 *****/
/************************/
// 3x3 单位矩阵
#define I3 Eigen::MatrixXd::Identity(3, 3)

// 12x12 单位矩阵
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 单位矩阵
#define I18 Eigen::MatrixXd::Identity(18, 18)

/************************/
/****** 工具函数 *****/
/************************/

/**
 * 将12x1向量转换为3x4矩阵 (4条腿，每条腿3个自由度)
 * 输入:  [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4]^T
 * 输出: [[x1, x2, x3, x4],
 *          [y1, y2, y3, y4],
 *          [z1, z2, z3, z4]]
 */
inline Vec34 vec12ToVec34(const Vec12& vec12)
{
    Vec34 vec34;
    for(int i = 0; i < 4; ++i)
    {
        vec34.col(i) = vec12.segment(3*i, 3);
    }
    return vec34;
}

/**
 * 将3x4矩阵转换为12x1向量
 * 输入:  [[x1, x2, x3, x4],
 *          [y1, y2, y3, y4],
 *          [z1, z2, z3, z4]]
 * 输出: [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4]^T
 */
inline Vec12 vec34ToVec12(const Vec34& vec34)
{
    Vec12 vec12;
    for(int i = 0; i < 4; ++i)
    {
        vec12.segment(3*i, 3) = vec34.col(i);
    }
    return vec12;
}

/**
 * 从欧拉角(RPY)创建3D旋转矩阵 (ZYX约定)
 */
inline RotMat rpyToRotMat(double roll, double pitch, double yaw)
{
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);
    
    RotMat R;
    R(0,0) = cy*cp;
    R(0,1) = cy*sp*sr - sy*cr;
    R(0,2) = cy*sp*cr + sy*sr;
    
    R(1,0) = sy*cp;
    R(1,1) = sy*sp*sr + cy*cr;
    R(1,2) = sy*sp*cr - cy*sr;
    
    R(2,0) = -sp;
    R(2,1) = cp*sr;
    R(2,2) = cp*cr;
    
    return R;
}

/**
 * 将旋转矩阵转换为欧拉角(RPY) (ZYX约定)
 */
inline Vec3 rotMatToRpy(const RotMat& R)
{
    Vec3 rpy;
    rpy(1) = std::asin(-R(2,0));  // pitch
    rpy(0) = std::atan2(R(2,1), R(2,2));  // roll
    rpy(2) = std::atan2(R(1,0), R(0,0));  // yaw
    return rpy;
}

#endif  // MATHTYPES_H
