#include "fileio.hpp"
#include <absl/strings/str_split.h>
#include <fstream>
#include <iostream>

// #define FileIODebug

double FileIO::acc_scale = 1.5258789063E-06;
double FileIO::gry_scale = 1.0850694444E-07;
int FileIO::freq         = 100;

bool FileIO::getIMUdata(const string &imufile, vector<IMU> &imu_data, bool is_imu_increment) {
    if (imufile.substr(imufile.find_last_of(".") + 1, 3) != "ASC") {
        cerr << "文件名：" << imufile << " 错误，目前只处理ASC格式数据！" << endl;
        return false;
    }
    fstream ifs(imufile, ios::in);
    if (!ifs.is_open()) {
        cerr << "文件：" << imufile << " 打开失败！" << endl;
        return false;
    }
    string line;
    getline(ifs, line);
    line.replace(line.find(";"), 1, ",");
    line.replace(line.find("*"), 1, ",");
    vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(","), absl::SkipWhitespace());

    double wsec = stod(splits[4]); // IMU周内秒

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

        // 此处经过了轴系调整
        imu.dvel << -stod(splits[7]), stod(splits[8]), -stod(splits[6]);
        imu.dvel *= acc_scale; // 此时imu.dvel的值是速度增量

        imu.dtheta << -stod(splits[10]), stod(splits[11]), -stod(splits[9]);
        imu.dtheta *= gry_scale; // 此时的imu.dtheta的值角度增量

        if (!is_imu_increment) {
            imu.dvel *= freq;   // 转换为加速度
            imu.dtheta *= freq; // 转换为角速度
        }

        imu_data.emplace_back(imu);
    }
    return true;
}

bool FileIO::getGNSSdata(const string &gnssfile, vector<GNSS> &gnss_data) {
    if (gnssfile.substr(gnssfile.find_last_of(".") + 1, 3) != "pos") {
        cerr << "文件名：" << gnssfile << " 错误，目前只处理pos格式数据！" << endl;
        return false;
    }
    fstream ifs(gnssfile, ios::in);
    if (!ifs.is_open()) {
        cerr << "文件：" << gnssfile << " 打开失败！" << endl;
        return false;
    }
    string line;

    // 跳过前两行的文件头
    for (int i = 0; i < 2; i++) {
        getline(ifs, line);
    }
    getline(ifs, line);
    vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(" "), absl::SkipWhitespace());

    double wsec = stod(splits[1]);

    // 开始读取数据
    while (getline(ifs, line)) {
        // 读取GNSS数据文件
        vector<string> splits = absl::StrSplit(line, absl::ByAnyChar(" "), absl::SkipWhitespace());

        // 可能会出现一个历元多次采样的问题
        if (abs(stod(splits[1]) - wsec) < 1E-6) {
#ifdef FileIODebug
            cout.flags(ios::fixed);
            cout.precision(6);
            cout << "GNSS 重复历元！\n";
            cout << "上一历元的time：" << wsec << ",\t" << "当前历元的time：" << stod(splits[1]) << endl;
#endif
            continue;
        }

        // 解析GNSS数据
        GNSS gnss;
        gnss.week = stod(splits[0]);
        gnss.time = stod(splits[1]);
        wsec      = gnss.time;
        gnss.blh << stod(splits[2]) * D2R, stod(splits[3]) * D2R, stod(splits[4]); // BLH位置（rad）
        gnss.posstd << stod(splits[5]), stod(splits[6]), stod(splits[7]);          // NED位置标准差（m）
        gnss.vel << stod(splits[8]), stod(splits[9]), -stod(splits[10]);           // NED速度（m/s）
        gnss.velstd << stod(splits[11]), stod(splits[12]), stod(splits[13]);       // NED速度标准差（m/s）
        gnss_data.emplace_back(gnss);
    }
    return true;
}