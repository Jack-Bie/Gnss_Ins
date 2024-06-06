#pragma once
#include "types.hpp"
#include <string>
#include <vector>
using namespace std;

struct FileIO {
    static double acc_scale; // IMU的ASC数据转换为实际测量数据需要乘以转换因子
    static double gry_scale; // IMU的ASC数据转换为实际测量数据需要乘以转换因子
    static int freq;         // IMU的测量频率
    
    /**
     * @brief 读取ASC格式的IMU测量数据
     * 
     * @param [in] imufile IMU数据文件路径
     * @param [in,out] imu_data 存储读取的IMU测量数据 
     * @param is_imu_increment [defalt: true] 是否按速度增量和角度增量形式存储
     * @return true 读取文件成功
     * @return false 读取文件失败
     */
    static bool getIMUdata(const string &imufile, vector<IMU> &imu_data, bool is_imu_increment=true);

    /**
     * @brief 读取pos格式的GNSS-RTK测量结果
     * 
     * @param [in] gnssfile GNSS数据文件路径
     * @param [in,out] gnss_data 存储读取的GNSS-RTK测量结果
     * @return true 读取文件成功
     * @return false 读取文件失败
     */
    static bool getGNSSdata(const string &gnssfile, vector<GNSS> &gnss_data);
};