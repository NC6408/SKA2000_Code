# 设置CMake最低版本
cmake_minimum_required(VERSION 3.10)

# 设置项目名称
project(SKA2000_AI200G C)

# 设置C标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 设置目标系统
set(CMAKE_SYSTEM_NAME Linux)

# 设置目标架构
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 指定交叉编译器路径
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C Compiler ID: ${CMAKE_C_COMPILER_ID}")
message(STATUS "C Compiler Version: ${CMAKE_C_COMPILER_VERSION}")
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++ Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")

# 设置目标系统根目录
set(CMAKE_SYSROOT ~/ubuntu-jammy-arm64)
message("CMAKE_SYSROOT: ${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
message("CMAKE_FIND_ROOT_PATH: ${CMAKE_FIND_ROOT_PATH}")

# 只在目标系统目录中查找库
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# pthread
# 库文件
find_library(
    THREADS_LIBRARY 
    NAMES pthread
    PATHS ${CMAKE_SYSROOT}/usr/lib/
)
if (THREADS_LIBRARY)
    message(STATUS "Found pthread library at: ${THREADS_LIBRARY}")
else()
    message(FATAL_ERROR "pthread library not found")
endif()
# 头文件
find_path(
    THREADS_INCLUDE_DIR 
    NAMES pthread.h
    PATHS ${CMAKE_SYSROOT}/usr/include
)
if (THREADS_INCLUDE_DIR)
    message(STATUS "Found pthread.h at: ${THREADS_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "pthread.h not found")
endif()

# zlog
# 库文件
find_library(
    ZLOG_LIBRARY 
    NAMES zlog
    PATHS ${CMAKE_SYSROOT}/usr/local/lib
)
if (ZLOG_LIBRARY)
    message(STATUS "Found zlog library at: ${ZLOG_LIBRARY}")
else()
    message(FATAL_ERROR "zlog library not found")
endif()
# 头文件
find_path(
    ZLOG_INCLUDE_DIR 
    NAMES zlog.h
    PATHS ${CMAKE_SYSROOT}/usr/local/include
)
if (ZLOG_INCLUDE_DIR)
    message(STATUS "Found zlog.h at: ${ZLOG_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "zlog.h not found")
endif()

# xml2
find_package(LibXml2 REQUIRED)
if (LIBXML2_FOUND)
    message(STATUS "Found libxml2 library at: ${LIBXML2_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "libxml2 library not found")
endif()

# modbus
# 库文件
find_library(
    MODBUS_LIBRARY 
    NAMES modbus
    PATHS ${CMAKE_SYSROOT}/usr/local/lib
)
if (MODBUS_LIBRARY)
    message(STATUS "Found modbus library at: ${MODBUS_LIBRARY}")
else()
    message(FATAL_ERROR "modbus library not found")
endif()
# 头文件
find_path(
    MODBUS_INCLUDE_DIR 
    NAMES modbus.h
    PATHS ${CMAKE_SYSROOT}/usr/local/include/modbus
)
if (MODBUS_INCLUDE_DIR)
    message(STATUS "Found modbus.h at: ${MODBUS_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "modbus.h not found")
endif()

# cjson
# 库文件
find_library(
    CJSON_LIBRARY 
    NAMES cjson
    PATHS ${CMAKE_SYSROOT}/usr/local/lib
)
if (CJSON_LIBRARY)
    message(STATUS "Found cjson library at: ${CJSON_LIBRARY}")
else()
    message(FATAL_ERROR "cjson library not found")
endif()
# 头文件
find_path(
    CJSON_INCLUDE_DIR 
    NAMES cJSON.h
    PATHS ${CMAKE_SYSROOT}/usr/local/include/cjson
)
if (CJSON_INCLUDE_DIR)
    message(STATUS "Found cJSON.h at: ${CJSON_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "cJSON.h not found")
endif()

# 包含头文件
include_directories(
    # ${THREADS_INCLUDE_DIR}
    ${ZLOG_INCLUDE_DIR}
    ${LIBXML2_INCLUDE_DIR}
    ${MODBUS_INCLUDE_DIR}
    ${CJSON_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/lib/include/mongoose
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/comm_module
    ${CMAKE_SOURCE_DIR}/include/control_module
    ${CMAKE_SOURCE_DIR}/include/others
)

# 查找src目录中的所有源文件
file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/lib/src/*.c)

# 添加可执行文件
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 添加链接库
target_link_libraries(${PROJECT_NAME} 
    m                       # math
    # ${THREADS_LIBRARY}      # pthread
    ${ZLOG_LIBRARY}         # zlog
    ${LIBXML2_LIBRARIES}    # xml2
    ${MODBUS_LIBRARY}       # modbus
    ${CJSON_LIBRARY}        # cjson
)
