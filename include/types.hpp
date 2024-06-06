#pragma once
#include "Eigen/Dense"
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

static const double D2R = M_PI / 180.0;
static const double R2D = 180.0 / M_PI;


typedef struct GNSS {
    int week;        // GPS周
    double time;     // GPS周内秒
    Vector3d blh;    // GNSS BLH坐标系位置
    Vector3d posstd; // GNSS NED坐标系标准差
    Vector3d vel;    // GNSS NED坐标系速度
    Vector3d velstd; // GNSS NED坐标系速度标准差
    bool isvalid;
} GNSS;

typedef struct IMU {
    int week;        // IMU周
    double time;     // IMU周内秒
    double dt;       // IMU当前历元与前一历元的时间间隔
    Vector3d dtheta; // IMU当前历元输出的角度增量
    Vector3d dvel;   // IMU当前历元输出的速度增量

    void operator=(const IMU &right){
        week = right.week;
        time = right.time;
        dt = right.dt;
        dtheta = right.dtheta;
        dvel = right.dvel;
    }
} IMU;

typedef struct Attitude {
    Quaterniond qbn; // 姿态四元数
    Matrix3d cbn;    // 姿态矩阵
    Vector3d euler;  // 欧拉角 -> 横滚roll， 俯仰pitch， 航向yaw(heading)
} Attitude;

typedef struct PVA {
    Vector3d pos; // BLH系下的位置
    Vector3d vel; // NED系下的速度
    Attitude att; // 载体姿态
} PVA;

typedef struct ImuErr {
    Vector3d gyrbias;  // 陀螺零偏误差
    Vector3d accbias;  // 加速度计零偏误差
    Vector3d gyrscale; // 陀螺比例因子误差
    Vector3d accscale; // 加速度计比例因子误差
} ImuError;

// GNSS-INS 松组合需要维护的状态量，总共有21维
typedef struct NavState {
    PVA pav;           // 载体的 BLH位置、NED速度、姿态
    ImuError imuerror; // IMU传感器误差
} NavState;

typedef struct ImuNoise {
    Vector3d gyr_arw;      // 角度随机游走
    Vector3d acc_vrw;      // 速度随机游走
    Vector3d gyrbias_std;  // 陀螺零偏不稳定性
    Vector3d accbias_std;  // 加速度计零偏不稳定性
    Vector3d gyrscale_std; // 陀螺比例因子不稳定性
    Vector3d accscale_std; // 加速度计比例因子不稳定性
    double corr_time;      // 相关时间
} ImuNoise;