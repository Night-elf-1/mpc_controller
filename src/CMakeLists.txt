#aux_source_directory (. SRC_LIST)                               # aux_source_directory把当前目录下的源文件存列表存放到变量SRC_LIST里
#include_directories (../include)
#add_executable (main ${SRC_LIST})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)          # EXECUTABLE_OUTPUT_PATH ：目标二进制可执行文件的存放位置
                                                                # PROJECT_SOURCE_DIR：工程的根目录
                                                                # set的意思是把存放bin文件的位置，设置为工程根目录下的bin目录
set(CMAKE_CXX_STANDARD 14)
add_executable(CppAD_demo testcppad/CppAD_demo.cpp)
add_executable(cppad_ipopt_demo testcppad/cppad_ipopt_demo.cpp)
target_link_libraries(cppad_ipopt_demo ipopt)