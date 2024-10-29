cmake_minimum_required(VERSION 3.10)
project(MyMPCProject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 寻找依赖库Eigen
find_package(Eigen3 REQUIRED)

# 寻找依赖库osqp和OsqpEigen
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PythonLibs REQUIRED)

# 添加include目录
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OSQP_INCLUDE_DIRS}
    ${OSQPEIGEN_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PYTHON_INCLUDE_DIRS}
    /home/hamster/mycode/mpc_controller/include
    #/usr/include/python3.10
)

# 指定编译输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# 指定源文件
set(testmatplotlib src/test_matplotlib/testmatplotlib.cpp)
set(mpc_speed_steel 
    src/MPC_SPEED_STEEL_CONTROL/main.cpp
    src/MPC_SPEED_STEEL_CONTROL/modelpredictivecontrol.cpp
)

# 添加源码目录
add_executable(main src/main.cpp src/kinematic.cpp src/mpc_controller.cpp src/referencePath.cpp)
add_executable(mpc_speed_steel_ ${mpc_speed_steel})
add_executable(testdraw ${testmatplotlib})

# 链接依赖库
target_link_libraries(main 
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
    ${PYTHON_LIBRARIES}
    python3.10.so
    pthread
    dl
)

target_link_libraries(testdraw
    ${PYTHON_LIBRARIES}
    python3.10.so
    pthread
    dl
)

target_link_libraries(mpc_speed_steel_
    Eigen3::Eigen
    osqp::osqp
    OsqpEigen::OsqpEigen
    ${PYTHON_LIBRARIES}
    python3.10.so
    pthread
    dl
)

# 排除不需要编译的文件
set_source_files_properties(src/matlab_mpc/* PROPERTIES HEADER_FILE_ONLY TRUE)
set_source_files_properties(src/testcppad/* PROPERTIES HEADER_FILE_ONLY TRUE)

# 其它可选项
# 可以在这里定义更多的CMake选项，比如编译标志、调试信息等
