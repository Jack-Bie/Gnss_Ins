#pragma once
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

/* WGS84椭球模型参数
   NOTE:如果使用其他椭球模型需要修改椭球参数 */
const double WGS84_WIE = 7.2921151467E-5;       /// 地球自转角速度
const double WGS84_F   = 0.0033528106647474805; /// 扁率
const double WGS84_RA  = 6378137.0000000000;    /// 长半轴a
const double WGS84_RB  = 6356752.3142451793;    /// 短半轴b
const double WGS84_GM0 = 398600441800000.00;    /// 地球引力常数
const double WGS84_E1  = 0.0066943799901413156; /// 第一偏心率平方
const double WGS84_E2  = 0.0067394967422764341; /// 第二偏心率平方

class Earth {
public:
    /// 正常重力计算
    static double gravity(const Vector3d &blh) {
        double sin2 = sin(blh[0]);
        sin2 *= sin2;
        return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
               blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
    }

    /// 计算子午圈半径和卯酉圈半径
    static Vector2d getRmRn(double lat) {
        double tmp, sqrttmp;
        tmp = sin(lat);
        tmp *= tmp;
        tmp     = 1 - WGS84_E1 * tmp;
        sqrttmp = sqrt(tmp);
        return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
    }

    /// 计算地球自转角速度向量
    static Vector3d getWie(double lat) {
        return {WGS84_WIE * std::cos(lat), 0.0, -WGS84_WIE * std::sin(lat)};
    }

    /// 计算位移角速度向量
    static Vector3d getWen(double Ve, double Vn, double lat, double h) {
        Vector2d RmRn = getRmRn(lat);
        return {Ve / (RmRn(1) + h), -Vn / (RmRn(0) + h), -Ve * std::tan(lat) / (RmRn(1) + h)};
    }
};
