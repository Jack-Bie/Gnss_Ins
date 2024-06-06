#pragma once
#include <Eigen/Geometry>
#include <iostream>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation {

public:
    // 方向余弦矩阵转四元数
    static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
        return Quaterniond(matrix);
    }

    // 四元数转方向余弦矩阵
    static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
        return quaternion.toRotationMatrix();
    }

    // ZYX旋转顺序, 前右下的IMU, 输出RPY（roll, pitch, yaw）
    static Vector3d matrix2euler(const Eigen::Matrix3d &dcm) {
        Vector3d euler;

        euler[1] = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

        if (dcm(2, 0) <= -0.999) {
            euler[0] = 0;
            euler[2] = atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
        } else if (dcm(2, 0) >= 0.999) {
            euler[0] = 0;
            euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
            std::cout << "[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!" << std::endl;
        } else {
            euler[0] = atan2(dcm(2, 1), dcm(2, 2));
            euler[2] = atan2(dcm(1, 0), dcm(0, 0));
        }

        // yaw 0~2PI
        // if (euler[2] < 0) {
        //     euler[2] = M_PI * 2 + euler[2];
        // }
        return euler;  // yaw -PI~PI
    }

    static Vector3d quaternion2euler(const Quaterniond &quaternion) {
        return matrix2euler(quaternion.toRotationMatrix());
    }

    static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
        double angle = rotvec.norm();
        Vector3d vec = rotvec.normalized();
        return Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    static Matrix3d rotvec2matrix(const Vector3d &rotvec){
        double angle = rotvec.norm();
        Matrix3d dcm = Eigen::Matrix3d::Identity() + sin(angle)/angle * skewSymmetric(rotvec)
                        + (1-cos(angle))/(angle*angle) * skewSymmetric(rotvec) * skewSymmetric(rotvec);
        return dcm;
    }

    static Vector3d quaternion2vector(const Quaterniond &quaternion) {
        Eigen::AngleAxisd axisd(quaternion);
        return axisd.angle() * axisd.axis();
    }

    // RPY --> C_b^n, ZYX顺序
    static Matrix3d euler2matrix(const Vector3d &euler) {
        return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    static Quaterniond euler2quaternion(const Vector3d &euler) {
        return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    /// 向量的反对陈矩阵
    static Matrix3d skewSymmetric(const Vector3d &vector) {
        Matrix3d mat;
        mat << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
        return mat;
    }

    /// 左乘四元数对应的矩阵
    static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + skewSymmetric(q.vec());
        return ans;
    }

    /// 右乘四元数对应的矩阵
    static Eigen::Matrix4d quaternionright(const Quaterniond &p) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = p.w();
        ans.block<1, 3>(0, 1) = -p.vec().transpose();
        ans.block<3, 1>(1, 0) = p.vec();
        ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - skewSymmetric(p.vec());
        return ans;
    }
};
