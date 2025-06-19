# 设置目标系统
set(CMAKE_SYSTEM_NAME Linux)

# 设置目标架构
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 设置目标系统根目录
set(CMAKE_SYSROOT /home/zy/aarch64_sysroot)

# 设置pkg-config环境变量
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${CMAKE_SYSROOT})
set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig)

# 指定交叉编译器路径
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 只在目标系统目录中查找库
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
