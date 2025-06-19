#define _POSIX_C_SOURCE 200809L // 启用CLOCK_MONOTONIC
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <modbus.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <stdint.h>  // 用于定义固定大小的数据类型


#include "SKA_SendToPlcThread.h"
#include "SKA_Logger.h"
#include "SKA_ThreadSafety.h"
#include "SKA_CommunicationInterface.h"
#include "SKA_Configurer.h"
#include "SKA_ControlThread.h"

#define REGISTER_NUM 128 // 寄存器数量
#define PERIOD_S 0.1 // 采样周期

// 服务器和客户端地址
static struct sockaddr_in stServerAddress, stClientAddress;

// 字节缓冲区
static SKA_ComAntiswayControllerOutput stByteBuffer;

// 寄存器缓冲区
static uint16_t arrRegBuffer[REGISTER_NUM];

/**
 * @brief 使用TCP协议向客户端发送数据
 * @return void
 */
static void SKA_SendToPlcThread_Tcp();

/**
 * @brief 使用UDP协议向客户端发送数据
 * @return void
 */
static void SKA_SendToPlcThread_Udp();

/**
 * @brief 使用Modbus TCP协议向PLC发送数据
 * @return void
 */
static void SKA_SendToPlcTHread_ModbusTcp();

/**
 * @brief 准备客户端数据
 * @return void
 */
static void SKA_SendToPlcThread_PrepareData();

void *SKA_SendToPlcThread_Run(void *arg)
{
    // 判断通信协议
    if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "TCP") == 0)
    {   
        // TCP
        zlog_info(pCategory, "Using TCP protocol to send data to PLC");
        SKA_SendToPlcThread_Tcp();
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "UDP") == 0)
    {
        // UDP
        zlog_info(pCategory, "Using UDP protocol to send data to PLC");
        SKA_SendToPlcThread_Udp();
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "Modbus-TCP") == 0)
    {
        // Modbus TCP
        zlog_info(pCategory, "Using Modbus-TCP protocol to send data to PLC");
        SKA_SendToPlcTHread_ModbusTcp();
    }
    else
    {   
        // 无效
        zlog_error(pCategory, "Invalid PLC communication protocol");
        exit(EXIT_FAILURE);
    }

    return NULL;
}

static void SKA_SendToPlcThread_Tcp()
{
    // 创建socket文件描述符
    int nServerFd;
    if ((nServerFd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        zlog_error(pCategory, "Socket failed");
        exit(EXIT_FAILURE);
    }

    // 设置地址复用
    int nOpt = 1;
    if (setsockopt(nServerFd, SOL_SOCKET, SO_REUSEADDR, &nOpt, sizeof(nOpt)))
    {
        zlog_error(pCategory, "Setsockopt failed");
        exit(EXIT_FAILURE);
    }

    // 设置服务器套接字为阻塞模式
    int nFlags = fcntl(nServerFd, F_GETFL, 0);
    if (nFlags == -1)
    {
        zlog_error(pCategory, "F_GETFL failed");
        exit(EXIT_FAILURE);
    }
    if (fcntl(nServerFd, F_SETFL, nFlags & ~O_NONBLOCK) == -1)
    {
        zlog_error(pCategory, "F_SETFL failed");
        exit(EXIT_FAILURE);
    }

    // 填充服务器和客户端地址
    memset(&stServerAddress, 0, sizeof(stServerAddress));
    memset(&stClientAddress, 0, sizeof(stClientAddress));

    // 设置服务器协议族、地址、端口
    stServerAddress.sin_family = AF_INET;
    stServerAddress.sin_addr.s_addr = inet_addr(stConfiguration.stCommConfig.ASC_ControlNetPortIp);
    stServerAddress.sin_port = htons(stConfiguration.stCommConfig.ASC_SendPortToPlc);

    // 绑定
    if (bind(nServerFd, (struct sockaddr *)&stServerAddress, sizeof(stServerAddress)) < 0)
    {
        zlog_error(pCategory, "Bind failed");
        exit(EXIT_FAILURE);
    }

    // 监听
    if (listen(nServerFd, 3) < 0)
    {
        zlog_error(pCategory, "Listen failed");
        exit(EXIT_FAILURE);
    }

    zlog_info(
        pCategory, 
        "Send_PLC_Data TCP Server is listening: %s:%d", 
        inet_ntoa(stServerAddress.sin_addr), ntohs(stServerAddress.sin_port)
    );

    while (1)
    {
        int nClientSocket;
        int nAddrLen = sizeof(stClientAddress);
        
        if ((nClientSocket = accept(nServerFd, (struct sockaddr *)&stClientAddress, (socklen_t *)&nAddrLen)) < 0)
        {
            zlog_error(pCategory, "Accept failed");
            continue;
        }
        // 建立连接
        bIsSendDataFault = false;
        // 打印客户端的IP地址和端口号
        char *strClientIp = inet_ntoa(stClientAddress.sin_addr);
        int nClientPort = ntohs(stClientAddress.sin_port);
        zlog_info(pCategory, "Client connected: %s:%d", strClientIp, nClientPort);

        // 设置客户端套接字为阻塞模式
        int nFlags = fcntl(nClientSocket, F_GETFL, 0);
        if (nFlags == -1)
        {
            zlog_error(pCategory, "F_GETFL failed");
            exit(EXIT_FAILURE);
        }
        if (fcntl(nClientSocket, F_SETFL, nFlags & ~O_NONBLOCK) == -1)
        {
            zlog_error(pCategory, "F_SETFL failed");
            exit(EXIT_FAILURE);
        }


        int nFlag = 1;
        if (setsockopt(nClientSocket, IPPROTO_TCP, TCP_NODELAY, (char*)&nFlag, sizeof(nFlag)))
        {
            zlog_error(pCategory, "Set client sock opt  TCP_NODELAY failed");
            exit(EXIT_FAILURE);
        }

        // 记录时间
        struct timespec stNext;
        // CLOCK_MONOTONIC时钟用于获取系统的单调时间, 从某个固定点（通常是系统启动时间）开始的时间，不受系统时间调整的影响
        if (clock_gettime(CLOCK_MONOTONIC, &stNext) != 0)
        {
            zlog_error(pCategory, "Failed to get current time");
            exit(EXIT_FAILURE);
        }

        // 周期性发送数据
        while (1)
        {
            // 设置防摇控制器IP
            stComAscOutput.stSendPacketHeader.serverIpSuffix = stComAscInput.stReceivePacketHeader.serverIpSuffix;
            // printf("stSendPacketHeader.serverIpSuffix: %d\n", stComAscOutput.stSendPacketHeader.serverIpSuffix);

            // 设置报文数据大小
            int nBytesSent = send(nClientSocket, (void *)&stByteBuffer, sizeof(stByteBuffer), MSG_WAITALL);
            stComAscOutput.stSendPacketHeader.dataLength = nBytesSent;
            // printf("dataLength: %d\n", stComAscOutput.stSendPacketHeader.dataLength);
            // 准备客户端数据
            SKA_SendToPlcThread_PrepareData();

            // 发送数据
            
            if (nBytesSent < 0)
            {
                zlog_error(pCategory, "Failed to send data: %s", strerror(errno));
                // 断开当前连接
                break;
            }
           // zlog_debug(pCategory, "Sent %d bytes to %s:%d", nBytesSent, strClientIp, nClientPort);

            // 发送完成时刻
            struct timespec stTime;
            if (clock_gettime(CLOCK_MONOTONIC, &stTime) != 0)
            {
                zlog_error(pCategory, "Failed to get current time");
                exit(EXIT_FAILURE);
            }
            zlog_debug(pCategory, "Time: %ld.%09lds", stTime.tv_sec, stTime.tv_nsec);

            // 计算下一个周期的时间点
            stNext.tv_nsec += (long)(PERIOD_S * 1000000000L);
            while (stNext.tv_nsec >= 1000000000L)
            {
                stNext.tv_nsec -= 1000000000L;
                stNext.tv_sec++;
            }

            // 睡眠直到下一个周期
            if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stNext, NULL) != 0)
            {
                zlog_error(pCategory, "Failed to sleep");
                exit(EXIT_FAILURE);
            }
        }

        // 断开连接
        bIsSendDataFault = false;

        // 关闭套接字
        if (close(nClientSocket) < 0)
        {
            zlog_error(pCategory, "Failed to close client_socket");
            exit(EXIT_FAILURE);
        }
    }

    // 关闭套接字
    if (close(nServerFd) < 0)
    {
        zlog_error(pCategory, "Failed to close nServerFd");
        exit(EXIT_FAILURE);
    }
}

static void SKA_SendToPlcThread_Udp()
{
    // 创建socket文件描述符
    int nServerFd;
    if ((nServerFd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
        zlog_error(pCategory, "Socket failed");
        exit(EXIT_FAILURE);
    }

    // 设置地址复用
    int nOpt = 1;
    if (setsockopt(nServerFd, SOL_SOCKET, SO_REUSEADDR, &nOpt, sizeof(nOpt)))
    {
        zlog_error(pCategory, "Setsockopt failed");
        exit(EXIT_FAILURE);
    }

    // 设置服务器套接字为阻塞模式
    int nFlags = fcntl(nServerFd, F_GETFL, 0);
    if (nFlags == -1)
    {
        zlog_error(pCategory, "F_GETFL failed");
        exit(EXIT_FAILURE);
    }
    if (fcntl(nServerFd, F_SETFL, nFlags & ~O_NONBLOCK) == -1)
    {
        zlog_error(pCategory, "F_SETFL failed");
        exit(EXIT_FAILURE);
    }

    // 填充服务器和客户端地址
    memset(&stServerAddress, 0, sizeof(stServerAddress));
    memset(&stClientAddress, 0, sizeof(stClientAddress));
    
    // 设置服务器协议族、地址、端口
    stServerAddress.sin_family = AF_INET;
    stServerAddress.sin_addr.s_addr = inet_addr(stConfiguration.stCommConfig.ASC_ControlNetPortIp);
    stServerAddress.sin_port = htons(stConfiguration.stCommConfig.ASC_SendPortToPlc);

    // 设置客户端协议族、地址、端口
    stClientAddress.sin_family = AF_INET;
    inet_pton(AF_INET, stConfiguration.stCommConfig.PLC_ControlNetPortIp, &stClientAddress.sin_addr);
    stClientAddress.sin_port = htons(stConfiguration.stCommConfig.PLC_RecvPortFromAsc);

    // 绑定
    if (bind(nServerFd, (struct sockaddr *)&stServerAddress, sizeof(stServerAddress)) < 0)
    {
        zlog_error(pCategory, "Bind failed");
        exit(EXIT_FAILURE);
    }

    zlog_info(
        pCategory, 
        "Send_PLC_Data UDP Server is ready to send on: %s:%d -> %s:%d", 
        inet_ntoa(stServerAddress.sin_addr), ntohs(stServerAddress.sin_port),
        inet_ntoa(stClientAddress.sin_addr), ntohs(stClientAddress.sin_port)
    );

    // 记录时间
    struct timespec stNext;
    // CLOCK_MONOTONIC时钟用于获取系统的单调时间, 从某个固定点（通常是系统启动时间）开始的时间，不受系统时间调整的影响
    if (clock_gettime(CLOCK_MONOTONIC, &stNext) != 0)
    {
        zlog_error(pCategory, "Failed to get current time");
        exit(EXIT_FAILURE);
    }

    // 周期性发送数据
    while (1)
    {
        // 准备客户端数据
        SKA_SendToPlcThread_PrepareData();

        // 设置防摇控制器IP
        stComAscOutput.stSendPacketHeader.serverIpSuffix = stComAscInput.stReceivePacketHeader.serverIpSuffix;

        // 设置报文数据大小
        int nBytesSent = sendto(nServerFd, (void *)&stByteBuffer, sizeof(stByteBuffer), 0, (struct sockaddr *)&stClientAddress, sizeof(stClientAddress));
        stComAscOutput.stSendPacketHeader.dataLength = nBytesSent;

        if (nBytesSent >= 0)
        {
            // 通信正常
            bIsSendDataFault = false;

            // zlog_debug(
            //     pCategory, 
            //     "Sent %d bytes to %s:%d.", 
            //     nBytesSent, inet_ntoa(stClientAddress.sin_addr), ntohs(stClientAddress.sin_port)
            // );
        }
        else
        {
            // 通信故障
            bIsSendDataFault = true;
            
            zlog_error(pCategory, "Failed to send data: %s", strerror(errno));
        }

        // 发送完成时刻
        struct timespec stTime;
        if (clock_gettime(CLOCK_MONOTONIC, &stTime) != 0)
        {
            zlog_error(pCategory, "Failed to get current time");
            exit(EXIT_FAILURE);
        }
       // zlog_debug(pCategory, "Time: %ld.%09lds", stTime.tv_sec, stTime.tv_nsec);

        // 计算下一个周期的时间点
        stNext.tv_nsec += (long)(PERIOD_S * 1000000000L);
        while (stNext.tv_nsec >= 1000000000L)
        {
            stNext.tv_nsec -= 1000000000L;
            stNext.tv_sec++;
        }

        // 睡眠直到下一个周期
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stNext, NULL) != 0)
        {
            zlog_error(pCategory, "Failed to sleep");
            exit(EXIT_FAILURE);
        }
    }

    // 关闭套接字
    if (close(nServerFd) < 0)
    {
        zlog_error(pCategory, "Failed to close nServerFd");
        exit(EXIT_FAILURE);
    }
}

static void SKA_SendToPlcTHread_ModbusTcp()
{
    while (1)
    {
        // 初始化上下文
        const char *strSlaveIp = stConfiguration.stCommConfig.ASC_ControlNetPortIp;     // 从站IP
        int nPort = stConfiguration.stCommConfig.ASC_SendPortToPlc;                     // 从站端口
        modbus_t *pCtx = modbus_new_tcp(strSlaveIp, nPort);
        if (pCtx == NULL) {
            zlog_error(pCategory, "Failed to create the libmodbus context: %s", modbus_strerror(errno));
            exit(EXIT_FAILURE);
        }

        // 连接从站
        if (modbus_connect(pCtx) == -1) {
            zlog_error(pCategory, "Failed to connect slave: %s. Reconneting...", modbus_strerror(errno));
            modbus_close(pCtx);
            modbus_free(pCtx);
            continue;
        }

        // 建立连接
        bIsSendDataFault = false;

        zlog_info(pCategory, "Connected to slave: %s:%d", strSlaveIp, nPort);

        // 记录时间
        struct timespec stNext;
        // CLOCK_MONOTONIC时钟用于获取系统的单调时间, 从某个固定点（通常是系统启动时间）开始的时间，不受系统时间调整的影响
        if (clock_gettime(CLOCK_MONOTONIC, &stNext) != 0)
        {
            zlog_error(pCategory, "Failed to get current time");
            exit(EXIT_FAILURE);
        }

        // 持续发送数据
        while (1)
        {
            // 准备客户端数据
            SKA_SendToPlcThread_PrepareData();

            // 发送数据
            int nRegisterNum = sizeof(SKA_ComAntiswayControllerOutput) / 2;
            int nSendRegisters = modbus_write_registers(pCtx, 0, nRegisterNum, arrRegBuffer);
            if (nSendRegisters == -1) {
                // 如果写入失败，尝试重新连接
                zlog_error(pCategory, "Failed to write data: %s. Reconneting...", modbus_strerror(errno));
                break;
            }
            zlog_debug(pCategory, "Sent %d registers to slave: %s:%d", nSendRegisters, strSlaveIp, nPort);

            // 发送完成时刻
            struct timespec stTime;
            if (clock_gettime(CLOCK_MONOTONIC, &stTime) != 0)
            {
                zlog_error(pCategory, "Failed to get current time");
                exit(EXIT_FAILURE);
            }
            zlog_debug(pCategory, "Time: %ld.%09lds", stTime.tv_sec, stTime.tv_nsec);

            // 计算下一个周期的时间点
            stNext.tv_nsec += (long)(PERIOD_S * 1000000000L);
            while (stNext.tv_nsec >= 1000000000L)
            {
                stNext.tv_nsec -= 1000000000L;
                stNext.tv_sec++;
            }

            // 睡眠直到下一个周期
            if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stNext, NULL) != 0)
            {
                zlog_error(pCategory, "Failed to sleep");
                exit(EXIT_FAILURE);
            }
        }

        // 断开连接
        bIsSendDataFault = true;

        modbus_close(pCtx);
        modbus_free(pCtx);
    }
}

static void SKA_SendToPlcThread_PrepareData()
{
    // 获取PLC数据互斥锁
    if (pthread_mutex_lock(&unPlcDataMutex) != 0)
    {
        zlog_error(pCategory, "Failed to lock unPlcDataMutex");
        // 尝试再次获取锁
        if (pthread_mutex_lock(&unPlcDataMutex) != 0)
        {
            zlog_error(pCategory, "Failed to lock unPlcDataMutex again");
            exit(EXIT_FAILURE);
        }
    }


    // 设置心跳
    stComAscOutput.stGeneralOutput.nAscHeartbeat = stComAscInput.stGeneralInput.nPlcHeartbeat + 1;

    // 序列化
    if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "TCP") == 0 || strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "UDP") == 0)
    {
        if (SKA_ComInt_OutputH2N(&stComAscOutput, &stByteBuffer) != 0)
        {
            zlog_error(pCategory, "Failed to serialize data");
            exit(EXIT_FAILURE);
        }
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "Modbus-TCP") == 0)
    {
        if (SKA_ComInt_OutputSerialize(&stComAscOutput, arrRegBuffer, REGISTER_NUM) != 0)
        {
            zlog_error(pCategory, "Failed to serialize data");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        zlog_error(pCategory, "Invalid PLC communication protocol");
        exit(EXIT_FAILURE);
    }

        // 显示输入
    // SKA_ComInt_ShowOutput(&stComAscOutput);
    // printf("----------\n");

    // 释放PLC数据互斥锁
    if (pthread_mutex_unlock(&unPlcDataMutex) != 0)
    {
        zlog_error(pCategory, "Failed to unlock unPlcDataMutex");
        // 尝试再次释放锁
        if (pthread_mutex_unlock(&unPlcDataMutex) != 0)
        {
            zlog_error(pCategory, "Failed to unlock unPlcDataMutex again");
            exit(EXIT_FAILURE);
        }
    }
}