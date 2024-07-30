# GINS 项目说明文档

## 1. 项目介绍

### 1.1 可执行程序说明

本项目是在MacOS系统基于CMake工具构建完成的，整个项目我已经编译好了，可执行文件都放在`bin`目录下，其中`GINS`是最终的可执行程序。

其他几个如`att_test`、`insmech`、`tools`，是当时为了完成一些次要任务创建的可执行程序，比如姿态正确性的验证、机械编排正确性的验证等。

所有可执行文件的源码放在`app`目录下，其中`insmech_test.cpp`会找不到`insmech.hpp`头文件，因为我在验证完机械编排正确后，将此文的实现整合到`gins.cpp`文件中了，然后就删除了`insmech.hpp`文件。如果想尝试编译`insmech_test.cpp`，可以利用`git`工具将本项目回退到之前的版本。

### 1.2 可执行文件的使用说明

下面以`GINS`这个可执行程序，说明其使用方法。
1. 首先进入到本工程的根目录
2. 执行`./bin/GINS --help`查看程序帮助文档
3. 上述命令会输出如下结果
```shell
GNSS-INS松组合程序使用方法如下：
	./bin/GINS ./dataset/gins.yaml

使用说明：GINS可执行程序必须且只接受一个命令行参数，该参数是一个yaml配置文件，可参考./dataset/gins.yaml文件进行修改
可以添加至多一个flag选项控制输出文件格式！
Usage: ./bin/GINS [OPTIONS] config_path

Positionals:
  config_path TEXT REQUIRED   输入配置yaml文件

Options:
  -h,--help                   Print this help message and exit
  -s,--rts                    是否进行RTS平滑
```
4. 解算数据，以`./dataset/20240522/playground/playground.yaml`文件为例，这个配置文件配置的是我们小组小推车实验的操场轨迹数据
```shell
# 不进行RTS平滑处理
./bin/GINS ./dataset/20240522/playground/playground.yaml

# 进行RTS平滑处理
./bin/GINS -s ./dataset/20240522/playground/playground.yaml

# 或者
./bin/GINS --rts ./dataset/20240522/playground/playground.yaml
```
输出文件会放在yaml配置文件所描述的位置。

5. 输出文件说明
带有`rts`命名的文件是经过RTS平滑处理后的输出文件。

- `blhres.txt`：输出BLH结果，可以直接拖到RTKLib的`rtkplot.exe`完成绘图操作
- `navres.txt`：输出ENU的位置、速度、姿态信息，用于Python绘制轨迹图、进行结果数据的分析等
- `navresstd.txt`：输出ENU的位置、速度、姿态标准差，用于查看结果的精度信息
- `navxyz.txt`：输出XYZ的位置、速度及其标准差，用于与PosMind的结果对比分析

## 2. 工程项目结构说明

```shell
GINS
├── CMakeLists.txt           // CMake 编译配置文件
├── PythonCode               // 对实验结果进行数据分析的Python代码文件夹
├── README.md                // 项目说明文档
├── app                      // 存放生成可执行程序的cpp文件的文件夹
├── bin                      // 存放生成的可执行程序，以及库文件
├── build                    // CMake编译项目生成的中间结果
├── dataset                  // 存放实验数据的文件夹
├── include                  // 存放C++工程项目源码的头文件的文件夹
├── src                      // 存放C++工程项目源码的源文件的文件夹
└── 综合实习松组合实验报告.pdf  // 实习报告文件
```

项目头文件目录说明
```shell
include
├── earth.hpp      // 定义地理参数常量
├── fileio.hpp     // 文件读写相关的类
├── gins.hpp       // 松组合算法实现的主要类
├── init.hpp       // 与初始对准相关的函数
├── rotation.hpp   // 姿态转换相关的类
└── types.hpp      // 实现松组合算法的基本数据结构的定义
```

要编译本项目需要安装 `CMake` 和 `Make` 工具，同时在系统中通过 CMake 工具安装如下的项目依赖库

- `Eigen3`：用于矩阵等运算的数学库
- `yaml-cpp`：用于解析YAML格式的配置文件的库
- `GTest`：Google开发的`abseil`库的依赖库
- `abseil`：这个库本身具有很多功能，项目中仅用于字符串的处理
- `CLI11`：用于配置命令行参数的库

以上的库都是开源的，在Github上都可以找到，利用CMake编译此工程项目前，确保在MacOS系统中已经成功安装这些库。库的安装步骤下面会举个例子说明。

## 3. CMake安装第三方库的步骤

下面以安装`yaml-cpp`这个第三方库为例，说明如何在MacOS系统中，通过CMake安装C++工程项目的第三方依赖库

1. 利用`git`工具克隆目标仓库
```shell
git clone --depth=1 git@github.com:jbeder/yaml-cpp.git
```
2. 进入`yaml-cpp`目录，执行`mkdir build`命令，创建一个`build`目录
3. 进入`build`目录，执行
```shell
cmake .. CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local
```
这个命令会将 yaml-cpp 的 release 版本编译到 build 目录下，同时指定 yaml-cpp 下载目录的前缀

4. 等待`cmake`命令执行完毕后，执行`make`命令，编译`yaml-cpp`库
5. 执行`sudo make install`命令，安装`yaml-cpp`库，这个库会被安装到`CMAKE_INSTALL_PREFIX`参数指定的目录，此处我们指定的是`/usr/local`目录
6. 安装完成后可以在MacOS系统的`/usr/local`找到所安装库的头文件和源文件目录

## 4. 编译项目

将所有这些库安装完成后，可以利用`VSCode`编译整个项目，如果没有预先配置VSCode，可以到网上找一篇文章，讲如何配置MacOS的VSCode的CMake开发环境，简单来说安装好CMake和Clang相关插件即可满足环境要求。


