#include "fileio.hpp"
#include "insmech.hpp"
#include "rotation.hpp"
#include <fstream>
#include <iostream>
#include <vector>
using namespace std;

// #define GINSDebug

// 接收两个路径参数，第一个是IMU观测文件路径，第二个是GNSS定位结果文件路径
int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "参数数量错误，请检查！\n需要接收两个路径参数，第一个是IMU观测文件（ASC）路径，第二个是GNSS定位结果文件"
                "（pos）路径"
             << endl;
        exit(-1);
    }

    string imufile = argv[1];
    string posfile = argv[2];

    vector<IMU> imu_data;          // 总的IMU观测数据
    vector<GNSS> gnss_data;        // 总的GNSS定位结果数据
    int imu_idx = 0, gnss_idx = 0; // 用于时间对齐的数据索引

    if (!FileIO::getIMUdata(imufile, imu_data)) {
        cerr << "IMU数据文件读取失败！" << endl;
        exit(-1);
    }
    if (!FileIO::getGNSSdata(posfile, gnss_data)) {
        cerr << "GNSS定位结果pos数据文件读取失败！" << endl;
        exit(-1);
    }

    PVA pvapre; // k-1 时刻位置、速度、姿态

    // 初始化
    while (imu_data[imu_idx].time < gnss_data[gnss_idx].time) {
#ifdef GINSDebug
        cout.flags(ios::fixed);
        cout.precision(6);
        cout << "imu time: " << imu_data[imu_idx].time << ",\t" << "gnss time: " << gnss_data[gnss_idx].time << endl;
#endif
        imu_idx++;
    }
    pvapre.pos = gnss_data[gnss_idx].blh;
    pvapre.vel = gnss_data[gnss_idx].vel;
    pvapre.att.euler << -0.387651 * D2R, 0.3049 * D2R, -87.5535 * D2R;
    pvapre.att.qbn = Rotation::euler2quaternion(pvapre.att.euler);
    pvapre.att.cbn = Rotation::euler2matrix(pvapre.att.euler);
#ifdef GINSDebug
    cout << "GNSS 位置：" << gnss_data[gnss_idx].blh.transpose() * R2D << endl;
    cout << "初始位置：" << pvapre.pos.transpose() << endl;
    cout << "初始速度：" << pvapre.vel.transpose() << endl;
    cout << "初始姿态：" << pvapre.att.euler.transpose() << endl;
#endif
    PVA pvacur; // k 时刻位置、速度、姿态
    pvacur = pvapre;

    IMU imupre; // k-1 时刻IMU输出数据
    IMU imucur; // k 时刻IMU输出数据

    fstream fout("result.txt", ios::out);
    cout << "\n***开始计算结果：***\n" << endl;

    for (; imu_idx < imu_data.size(); imu_idx++) {
        if (imu_data[imu_idx].time <= gnss_data[gnss_idx + 1].time &&
            imu_data[imu_idx + 1].time > gnss_data[gnss_idx + 1].time && false) {
            // 如果GNSS数据位于 k 时刻和 k+1 时刻之间，将当前位置更新为GNSS
            pvapre     = pvacur;
            pvacur.pos = gnss_data[gnss_idx].blh;
            pvacur.vel = gnss_data[gnss_idx].vel;
            gnss_idx++;
        } else {
            // 如果 k 时刻和 k+1 时刻之间没有GNSS数据，则进行机械编排更新
            imupre = imu_data[imu_idx - 1];
            imucur = imu_data[imu_idx];
            if (imucur.dvel.norm() < 1E-10 || imucur.dtheta.norm() < 1E-10) {
                continue;
            }
            INSMech::insMech(pvapre, pvacur, imupre, imucur);
            cout.flags(ios::fixed);
            cout.precision(8);
            cout << imucur.time << " " << pvacur.pos[0] * R2D << " " << pvacur.pos[1] * R2D << " " << pvacur.pos[2]
                 << " " << pvacur.vel[0] << " " << pvacur.vel[1] << " " << pvacur.vel[2] << " "
                 << pvacur.att.euler[0] * R2D << " " << pvacur.att.euler[1] * R2D << " " << pvacur.att.euler[2] * R2D
                 << endl;

            // 输出到result.txt文件
            fout.flags(ios::fixed);
            fout.precision(8);
            fout << imucur.time << " " << pvacur.pos[0] * R2D << " " << pvacur.pos[1] * R2D << " " << pvacur.pos[2]
                 << " " << pvacur.vel[0] << " " << pvacur.vel[1] << " " << pvacur.vel[2] << " "
                 << pvacur.att.euler[0] * R2D << " " << pvacur.att.euler[1] * R2D << " " << pvacur.att.euler[2] * R2D
                 << endl;
        }
    }
    cout << "结果输出在data文件夹的result.txt中！" << endl;
    fout.close();
    return 0;
}