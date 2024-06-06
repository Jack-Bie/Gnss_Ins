#include "CLI/CLI.hpp"
#include "absl/strings/str_format.h"
#include "earth.hpp"
#include "fileio.hpp"
#include "init.hpp"
#include "rotation.hpp"
#include <fstream>
#include <iostream>
using namespace std;
using Eigen::Matrix3d;

/**
 * @brief 计算初始对准结果，注意修改 start_idx, end_idx, phi
 *
 * @param imufile 用于初始对准的IMU观测数据
 */
void initAtt(const string &imufile, const int &start_idx = 0, const int &end_idx = 30000,
             const double &phi = 30.528297320436) {
    vector<IMU> imudata;
    cout << "IMU数据文件为: " << imufile << endl;
    FileIO::getIMUdata(imufile, imudata, false);
    cout << "总历元数: " << imudata.size() << endl;

    // 进行初始对准
    Vector3d initAtt = getInitAtt(imudata, start_idx, end_idx, phi);
    cout << "初始对准结果为[roll, pitch, yaw]: " << initAtt.transpose() * R2D << endl;
}

/**
 * @brief 计算Allan方长结果
 *
 * @param imufile
 * @param outfile
 */
void initAllan(const string &imufile, const string &outfile) {
    vector<IMU> imudata;
    cout << "IMU数据文件为: " << imufile << endl;
    getRawIMUdata(imufile, imudata);
    int size = imudata.size();
    cout << "总历元数: " << size << endl;
    vector<double> res_allan_std;

    if (outfile.substr(outfile.find_last_of(".") + 1, 3) != "txt") {
        cerr << "输出文件名必须为txt格式" << endl;
        exit(-1);
    }

    fstream fout(outfile, ios::out);
    constexpr absl::string_view format = "%-15.9lf ";

    for (int bins = 2; bins < 10000; bins += 10) {
        allanAnalysis(imudata, 0, size, bins, res_allan_std);
        string line = absl::StrFormat(format, res_allan_std[0]);
        for (int i = 1; i < res_allan_std.size(); i++) {
            absl::StrAppendFormat(&line, format, res_allan_std[i]);
        }
        cout << "bins = " << bins << "\t" << line << endl;
        fout << "bins=" << bins << "\t" << line << '\n';
    }
    fout.close();
}

int main(int argc, char *argv[]) {
    CLI::App app{"本程序提供静态解析粗对准和Allan方差分析功能，使用方法如下：\n"};
    // initAtt 子命令
    auto initAtt_cmd = app.add_subcommand("init", "静态解析粗对准功能");
    string imufile;
    int start_idx{0}, end_idx{30000};
    double phi = 30.528297320436;
    initAtt_cmd->add_option("imufile", imufile, "IMU ASC格式数据文件路径")->required();
    initAtt_cmd->add_option("-s,--start", start_idx, "指定开始历元索引")->default_val(0);
    initAtt_cmd->add_option("-e,--end", end_idx, "指定结束历元索引")->default_val(30000);
    initAtt_cmd->add_option("-p,--phi", phi, "指定IMU静止时所在的纬度值（deg）")->default_val(30.528297320436);

    auto initAllan_cmd = app.add_subcommand("allan", "Allan方差分析功能");
    string outfile;
    initAllan_cmd->add_option("imufile", imufile, "IMU ASC格式数据文件路径")->required();
    initAllan_cmd->add_option("outfile", outfile, "输出文件名(要求txt格式)")->required();

    CLI11_PARSE(app, argc, argv);

    if (initAtt_cmd->parsed()) {
        // cout << "imufile: " << imufile << "\nstart_idx: " << start_idx << "\nend_idx: " << end_idx << "\nphi: " <<
        // phi<< endl;
        initAtt(imufile, start_idx, end_idx, phi);
    } else if (initAllan_cmd->parsed()) {
        initAllan(imufile, outfile);
    } else {
        cout << app.help() << endl;
    }
    return 0;
}