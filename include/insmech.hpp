#pragma once
#include <Eigen/Geometry>
#include "types.hpp"

class INSMech {
public:
    /**
     * @brief INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
     * @param [in]     pvapre 上一时刻状态
     * @param [in,out] pvacur 输出当前时刻状态
     * @param [in]     imupre, imucur imudata
     * */
    static void insMech(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

private:
    /**
     * @breif 位置更新
     * */
    static void posUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

    /**
     * @breif 速度更新
     * */
    static void velUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);

    /**
     * @breif 姿态更新
     * */
    static void attUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur);
};
