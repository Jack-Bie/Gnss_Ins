#include "init.hpp"
#include "earth.hpp"
#include <absl/strings/str_split.h>
#include <fstream>

bool getRawIMUdata(const string &imufile, vector<IMU> &imudata) {
    if (imufile.substr(imufile.find_last_of(".") + 1, 3) != "ASC") {
        cerr << "文件名：" << imufile << " 错误，目前只处理ASC格式数据！" << endl;
        return false;
    }
    fstream ifs(imufile, ios::in);
    if (!ifs.is_open()) {
        cerr << "文件：" << imufile << " 打开失败！" << endl;
        return false;
    }
    double acc_scale = 1.5258789063E-06;
    double gyr_scale = 1.0850694444E-07;
    int freq         = 100;

    string line;
    getline(ifs, line);
    line.replace(line.find(";"), 1, ",");
    line.replace(line.find("*"), 1, ",");
    vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(","), absl::SkipWhitespace());

    double wsec = stod(splits[4]); // IMU周内秒
    fstream accx("AllanAccX.txt", ios::out), accy("AllanAccY.txt", ios::out), accz("AllanAccZ.txt", ios::out);
    fstream gyrx("AllanGyrX.txt", ios::out), gyry("AllanGyrY.txt", ios::out), gyrz("AllanGyrZ.txt", ios::out);
    while (getline(ifs, line)) {
        line.replace(line.find(";"), 1, ",");
        line.replace(line.find("*"), 1, ",");
        vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(","), absl::SkipWhitespace());

        // 可能会出现一个历元多次采样的问题
        if (abs(stod(splits[4]) - wsec) < 1E-6) {
#ifdef FileIODebug
            cout.flags(ios::fixed);
            cout.precision(6);
            cout << "IMU 重复历元，上一历元的time：" << wsec << ",\t" << "当前历元的time：" << stod(splits[4]) << endl;
#endif
            continue;
        }

        // 解析IMU数据
        IMU imu;
        imu.week = stod(splits[3]);
        imu.time = stod(splits[4]);
        imu.dt   = imu.time - wsec;
        wsec     = imu.time;
        imu.dvel << stod(splits[6]), -stod(splits[7]), stod(splits[8]);
        imu.dvel *= acc_scale * freq;
        accz << setiosflags(ios::fixed) << setprecision(16) << imu.dvel[0] << endl;
        accy << setiosflags(ios::fixed) << setprecision(16) << imu.dvel[1] << endl;
        accx << setiosflags(ios::fixed) << setprecision(16) << imu.dvel[2] << endl;

        imu.dtheta << stod(splits[9]), -stod(splits[10]), stod(splits[11]);
        imu.dtheta *= gyr_scale * freq;
        gyrz << setiosflags(ios::fixed) << setprecision(16) << imu.dtheta[0] << endl;
        gyry << setiosflags(ios::fixed) << setprecision(16) << imu.dtheta[1] << endl;
        gyrx << setiosflags(ios::fixed) << setprecision(16) << imu.dtheta[2] << endl;
        imudata.emplace_back(imu);
    }
    accz.close();
    accy.close();
    accx.close();
    gyrz.close();
    gyry.close();
    gyrx.close();
    return true;
}

void allanAnalysis(vector<IMU> &imudata, const int &start_idx, const int &end_idx, const int &bins,
                   vector<double> &res_allan_std) {
    res_allan_std.clear();
    vector<Vector3d> acc_mean; // 存储加速度计的X、Y、Z三轴的分块平均值向量
    acc_mean.reserve(bins);
    vector<Vector3d> gry_mean; // 存储加速度计的X、Y、Z三轴的分块平均值向量
    gry_mean.reserve(bins);
    Vector3d acc_tmp{0.0, 0.0, 0.0}, gry_tmp{0.0, 0.0, 0.0};
    int k    = (end_idx - start_idx) / bins; // 每一分块的长度
    int size = imudata.size();
    // 求每一分块的均值
    for (int i = 0; i < bins; i++) {
        for (int j = start_idx + i * k; (j < start_idx + i * (k + 1)) && (j + k < size); j++) {
            acc_tmp += imudata[j].dvel;
            gry_tmp += imudata[j].dtheta;
        }
        acc_mean.emplace_back(acc_tmp / k);
        gry_mean.emplace_back(gry_tmp / k);
        acc_tmp.setZero();
        gry_tmp.setZero();
    }

    // 根据平均值计算Allan方差
    Vector3d tmp;
    for (int i = 0; i < bins - 1; i++) {
        tmp = (acc_mean[i + 1] - acc_mean[i]).array().square();
        acc_tmp += tmp;
        tmp = (gry_mean[i + 1] - gry_mean[i]).array().square();
        gry_tmp += tmp;
    }
    acc_tmp = (acc_tmp / (2 * (bins - 1))).array().sqrt();
    gry_tmp = (gry_tmp / (2 * (bins - 1))).array().sqrt();
    for (auto &item : acc_tmp) {
        res_allan_std.emplace_back(item);
    }
    for (auto &item : gry_tmp) {
        res_allan_std.emplace_back(item);
    }
}

Vector3d getInitAtt(vector<IMU> &imudata, const int &start_idx, const int &end_idx, double phi, const double &&g) {
    Vector3d Wib_b(0.0, 0.0, 0.0);
    Vector3d g_b(0.0, 0.0, 0.0);
    for (int i = start_idx; i < end_idx; i++) {
        // 计算所有角速度的和用于求均值
        Wib_b += imudata[i].dtheta;
        g_b -= imudata[i].dvel;
    }
    Wib_b /= (end_idx - start_idx);
    g_b /= (end_idx - start_idx);

    Matrix3d A;
    A << 0, 0, 1, 0, 1, 0, 1, 0, 0;

    Matrix3d B;
    B.block(0, 0, 1, 3) = g_b.normalized().transpose();
    Vector3d tmp1       = g_b.cross(Wib_b);
    B.block(1, 0, 1, 3) = tmp1.normalized().transpose();
    Vector3d tmp2       = tmp1.cross(g_b);
    B.block(2, 0, 1, 3) = tmp2.normalized().transpose();
    // cout << "B = \n" << B << endl;
    Matrix3d Cb_n = A * B;
    cout << "Cb_n = \n" << Cb_n << endl;
    Vector3d initAtt = Rotation::matrix2euler(Cb_n);
    // Vector3d initAtt = Cb_n.eulerAngles(2,1,0);
    return initAtt;
}