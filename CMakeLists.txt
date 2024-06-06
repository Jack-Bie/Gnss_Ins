cmake_minimum_required(VERSION 3.10)
project(GINS)

# message("当前源代码目录：${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_STANDARD 20)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 查找第三方库
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTest REQUIRED)
find_package(absl REQUIRED)
find_package(CLI11 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC)
add_library(GinsLib SHARED ${SRC})

# 将第三方库的头文件导入到 GinsLib 中
message("GinsLib PUBLIC ${EIGEN3_INCLUDE_DIRS}")
target_include_directories(GinsLib PUBLIC ${EIGEN3_INCLUDE_DIRS})
message("GinsLib PUBLIC ${YAML-CPP_INCLUDE_DIRS}")
target_include_directories(GinsLib PUBLIC ${YAML_INCLUDE_DIRS})
message("GinsLib PUBLIC ${GTEST_INCLUDE_DIRS}")
target_include_directories(GinsLib PUBLIC ${GTEST_INCLUDE_DIRS})
message("GinsLib PUBLIC ${ABSL_INCLUDE_DIRS}")
target_include_directories(GinsLib PUBLIC ${ABSL_INCLUDE_DIRS})
message("GinsLib PUBLIC ${CLI11_INCLUDE_DIRS}")
target_include_directories(GinsLib PUBLIC ${CLI11_INCLUDE_DIRS})

# 链接第三方库
set(ABSL_PROPAGATE_CXX_STD true)
target_link_libraries(GinsLib PUBLIC
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  absl::strings
  absl::str_format
  absl::time
  CLI11::CLI11)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/app/main.cpp)
target_link_libraries(${PROJECT_NAME} GinsLib)

add_executable(tools ${PROJECT_SOURCE_DIR}/app/tools.cpp)
target_link_libraries(tools GinsLib)