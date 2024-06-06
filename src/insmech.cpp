#include "earth.hpp"
#include "rotation.hpp"

#include "insmech.hpp"


void INSMech::insMech(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {
    // 依次进行姿态更新、速度更新、位置更新, 不可调换顺序
    attUpdate(pvapre, pvacur, imupre, imucur);
    velUpdate(pvapre, pvacur, imupre, imucur);
    posUpdate(pvapre, pvacur, imupre, imucur);
}

void INSMech::attUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {
    // 计算 k-1 时刻地理参数
    Eigen::Vector2d RmRn;
    Eigen::Vector3d wie_n, wen_n;
    RmRn = Earth::getRmRn(pvacur.pos[0]);
    wie_n = Earth::getWie(pvacur.pos[0]);
    wen_n = Earth::getWen(pvacur.vel[1], pvacur.vel[0], pvacur.pos[0], pvacur.pos[2]);

    // 计算n系和b系的方向余弦矩阵 k-1 时刻到 k 时刻变换
    Vector3d phik = imucur.dtheta + imupre.dtheta.cross(imucur.dtheta) / 12.0;
    Vector3d zetak = (wie_n + wen_n) * imucur.dt;
    Matrix3d cbb = Rotation::rotvec2matrix(phik);
    Matrix3d cnn = Rotation::rotvec2matrix(-zetak);

    // 姿态更新完成
    pvapre.att.cbn = pvacur.att.cbn;    // pvapre的姿态从 k-2 时刻更新为 k-1 时刻
    pvapre.att.euler = pvacur.att.euler;
    pvacur.att.cbn = cnn * pvacur.att.cbn * cbb;    // pvacur的姿态从 k-1 时刻更新为 k 时刻
    pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
    pvacur.att.qbn = Rotation::euler2quaternion(pvacur.att.euler);
    return;
}

void INSMech::velUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl, midvel, midpos, wie_n, wen_n;
    Eigen::Vector3d temp1, temp2, temp3;
    Eigen::Matrix3d cnn, I33 = Eigen::Matrix3d::Identity();
    double gravity;

    // 计算地理参数，子午圈半径和卯酉圈半径，地球自转角速度投影到n系, n系相对于e系转动角速度投影到n系，重力值
    // k-2 时刻的地理参数
    // Eigen::Vector2d RmRn_pre = Earth::getRmRn(pvapre.pos[0]);
    Eigen::Vector3d wie_n_pre, wen_n_pre;
    wie_n_pre = Earth::getWie(pvapre.pos[0]);
    wen_n_pre = Earth::getWen(pvapre.vel[1], pvapre.vel[0], pvapre.pos[0], pvapre.pos[2]);
    double gravity_pre = Earth::gravity(pvapre.pos);

    // k-1 时刻的地理参数
    // Eigen::Vector2d RmRn_cur = Earth::getRmRn(pvacur.pos[0]);
    Eigen::Vector3d wie_n_cur, wen_n_cur;
    wie_n_cur = Earth::getWie(pvacur.pos[0]);
    wen_n_cur = Earth::getWen(pvacur.vel[1], pvacur.vel[0], pvacur.pos[0], pvacur.pos[2]);
    double gravity_cur = Earth::gravity(pvacur.pos);

    // k-1/2 时刻的地理参数和速度
    wie_n = 3.0/2.0 * wie_n_cur - 1.0/2.0 * wie_n_pre;
    wen_n = 3.0/2.0 * wen_n_cur - 1.0/2.0 * wen_n_pre;
    gravity = 3.0/2.0 * gravity_cur - 1.0/2.0 * gravity_pre;

    // // 先外推 k-1/2 时刻的位置，然后外推 k-1/2 时刻的地理参数
    // midpos[0] = pvacur.pos[0] + pvacur.vel[0]*imucur.dt/2.0;
    // midpos[1] = pvacur.pos[1] + pvacur.vel[1]*imucur.dt/2.0;
    // midpos[2] = pvacur.pos[2] - pvacur.vel[2]*imucur.dt/2.0;
    // wie_n = Earth::getWie(midpos[0]);
    // wen_n = Earth::getWen(midvel[1], midvel[0], midpos[0], midpos[2]);
    // gravity = Earth::gravity(midpos);
    midvel = 3.0/2.0 * pvacur.vel - 1.0/2.0 * pvapre.vel;

    // 旋转效应和双子样划桨效应
    temp1 = imucur.dtheta.cross(imucur.dvel) / 2.0;
    temp2 = imupre.dtheta.cross(imucur.dvel) / 12.0;
    temp3 = imupre.dvel.cross(imucur.dtheta) / 12.0;

    // b系比力积分项
    d_vfb = imucur.dvel + temp1 + temp2 + temp3;

    // 比力积分项投影到n系
    temp1 = (wie_n + wen_n) * imucur.dt;
    cnn   = I33 - 0.5*Rotation::skewSymmetric(temp1);
    d_vfn = cnn * pvapre.att.cbn * d_vfb;       // pvapre.att.cbn表示 k-1 时刻的姿态，因为此时已经完成了姿态的更新

    // 计算重力/哥式积分项
    gl << 0, 0, gravity;
    d_vgn = (gl - (2.0 * wie_n + wen_n).cross(midvel)) * imucur.dt;

    // 速度更新完成
    pvapre.vel = pvacur.vel;     // pvapre.vel从 k-2 时刻更新为 k-1 时刻
    pvacur.vel = pvacur.vel + d_vfn + d_vgn;    // pvacur.vel从 k-1 时刻更新为 k 时刻
    return;
}

void INSMech::posUpdate(PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {

    Eigen::Vector3d temp1, temp2, midvel;

    // 计算中间时刻的速度和位置
    midvel = (pvacur.vel + pvapre.vel) / 2.0;

    pvapre.pos = pvacur.pos;    // pvapre.pos从 k-2 时刻更新为 k-1 时刻

    // 计算中间时刻地理参数
    Eigen::Vector2d RmRn;
    Eigen::Vector3d wie_n, wen_n;
    RmRn = Earth::getRmRn(pvapre.pos[0]);

    pvacur.pos[2] = pvapre.pos[2] - midvel[2]*imucur.dt;    // pvacur.pos的高程从 k-1 更新到 k 时刻
    double midh = (pvacur.pos[2] + pvapre.pos[2])/2.0;
    pvacur.pos[0] = pvapre.pos[0] + midvel[0]/(RmRn(0)+midh)*imucur.dt;   // 纬度从 k-1 更新到 k 时刻
    double midphi = (pvacur.pos[0] + pvapre.pos[0])/2.0;
    RmRn = Earth::getRmRn(midphi);
    pvacur.pos[1] = pvapre.pos[1] + midvel[1]/((RmRn(1)+midh)*cos(midphi)) * imucur.dt;     // 经度从 k-1 更新到 k 时刻
    return;
}