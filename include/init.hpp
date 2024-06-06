#pragma once
#include "Eigen/Eigen"
#include "rotation.hpp"
#include "types.hpp"
#include <iostream>
#include <vector>
using Eigen::Vector3d;
using namespace std;

/**
 * @brief 根据静止采集的IMU数据进行Allan方差分析
 *
 * @param imudata 存储所有的IMU测量值
 * @param start_idx 用于Allan方差分析的历元起始索引
 * @param end_idx 用于Allan方差分析的历元结束索引
 * @param bins 将用于Allan方差分析的数据分成的份数
 * @param [in, out] res_allan_std 输出一组加速度计、陀螺仪的Allan标准差的值
 */
void allanAnalysis(vector<IMU> &imudata, const int &start_idx, const int &end_idx, const int &bins,
                   vector<double> &res_allan_std);

/**
 * @brief 读取IMU原始数据，输出加速度计和陀螺仪的三轴原始数据到txt文件
 *
 * @param imufile IMU原始数据的ASC文件
 * @param imudata 用于存储IMU原始数据的向量
 * @return true 成功读取并输出
 * @return false 读取失败
 */
bool getRawIMUdata(const string &imufile, vector<IMU> &imudata);

/**
 * @brief 采用初始静止的测量值做静态解析粗对准
 *
 * @param imudata 存储所有的IMU测量值
 * @param start_idx 用于静态解析粗对准的起始历元索引
 * @param end_idx 用于静态解析粗对准的结束历元索引
 * @param phi 静止点所在的纬度值，单位度
 * @param g [default: 9.7936174] 静止点的重力加速度，单位m/s^2
 * @return Vector3d 欧拉角，按[roll,pitch,yaw]顺序，单位度
 */
Vector3d getInitAtt(vector<IMU> &imudata, const int &start_idx, const int &end_idx, double phi,
                    const double &&g = 9.7936174);