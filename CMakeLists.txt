cmake_minimum_required (VERSION 3.21)
project (mpc_controller)
add_subdirectory (src)                      # 增加编译子目录
                                            # 指定src目录下存放了源文件，当执行cmake时，就会进入src目录下去找 src目录下的CMakeLists.txt
                                            # 在src目录下也建立一个CMakeLists.txt