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
#include <stdint.h>  // 用于定义固定大小的数据类型

#include "SKA_ReceiveFromPlcThread.h"
#include "SKA_Logger.h"
#include "SKA_ThreadSafety.h"
#include "SKA_CommunicationInterface.h"
#include "SKA_Configurer.h"

#define REGISTER_NUM 128 // 寄存器数量

// 服务器和客户端地址
static struct sockaddr_in stServerAddress, stClientAddress;

// 接收数据缓冲
static SKA_ComAntiswayControllerInput stComAscInputBuffer;

// Modbus映射
static modbus_mapping_t *pMbMapping;

/**
 * @brief 使用TCP协议接收客户端数据
 * @return void
 */
static void SKA_ReceiveFromPlcThread_Tcp();

/**
 * @brief 使用UDP协议接收客户端数据
 * @return void
 */
static void SKA_ReceiveFromPlcThread_Udp();

/**
 * @brief 使用Modbus TCP协议接收PLC数据
 * @return void
 */
static void SKA_ReceiveFromPlcThread_ModbusTcp();

/**
 * @brief 处理客户端数据
 * @return void
 */
static void SKA_ReceiveFromPlcThread_ProcessData();

void *SKA_ReceiveFromPlcThread_Run(void *arg)
{
    // 判断通信协议
    if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "TCP") == 0)
    {   
        // TCP
        zlog_info(pCategory, "Using TCP protocol to receive data from PLC");
        SKA_ReceiveFromPlcThread_Tcp();
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "UDP") == 0)
    {
        // UDP
        zlog_info(pCategory, "Using UDP protocol to receive data from PLC");
        SKA_ReceiveFromPlcThread_Udp();
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "Modbus-TCP") == 0)
    {
        // Modbus TCP
        zlog_info(pCategory, "Using Modbus-TCP protocol to receive data from PLC");
        SKA_ReceiveFromPlcThread_ModbusTcp();
    }
    else
    {   
        // 无效
        zlog_error(pCategory, "Invalid PLC communication protocol");
        exit(EXIT_FAILURE);
    }
    
    return NULL;
}

static void SKA_ReceiveFromPlcThread_Tcp()
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
    stServerAddress.sin_port = htons(stConfiguration.stCommConfig.ASC_RecvPortFromPlc);
    
    // 解析服务器IP后两位
    uint16_t serverIpLow;
    {
        unsigned int ip1, ip2, ip3, ip4;
        sscanf(stConfiguration.stCommConfig.ASC_ControlNetPortIp, "%u.%u.%u.%u", &ip1, &ip2, &ip3, &ip4);
        serverIpLow = (ip3 << 8) | ip4; // 组合 50.15 -> 0x3215
    }

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
        "Recv_PLC_Data TCP Server is listening: %s:%d", 
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
        bIsRecvDataFault = false;

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

        // 持续接收客户端数据
        while (1)
        {

            int nBytesReceived = recv(nClientSocket, (void *)&stComAscInputBuffer, sizeof(stComAscInputBuffer), MSG_WAITALL);

            if (nBytesReceived == sizeof(stComAscInputBuffer))
            {
                // 校验防摇控制器IP后两位
                 uint16_t receivedIpLow = ntohs(stComAscInputBuffer.stReceivePacketHeader.serverIpSuffix);
                if (receivedIpLow != serverIpLow)
                {
                    zlog_error(
                        pCategory,
                        "IP mismatch: expected 0x%04X, received 0x%04X",
                        serverIpLow, receivedIpLow);
                    continue;
                }
                // 校验报文数据长度
                uint16_t receivedDataLength = ntohs(stComAscInputBuffer.stReceivePacketHeader.dataLength);
                if (receivedDataLength != nBytesReceived)
                {
                    zlog_error(
                        pCategory, 
                        "Data length mismatch: expected %d, received %d",
                        nBytesReceived, receivedDataLength);
                    continue;
                }

                /* 处理客户端数据 */
                SKA_ReceiveFromPlcThread_ProcessData();

            }
            else if (nBytesReceived > 0)
            {
                // 部分接收：信号中断、连接终止、协议层异常
                zlog_error(pCategory, "Failed to receive full data: %s", strerror(errno));
                break;
            }
            else if (nBytesReceived == 0)
            {
                // 客户端主动关闭连接
                zlog_info(
                    pCategory, 
                    "Client has actively closed the connection: %s:%d", 
                    inet_ntoa(stClientAddress.sin_addr), ntohs(stClientAddress.sin_port));
                break;
            }
            else
            {
                // 接收数据失败
                zlog_error(pCategory, "Failed to receive data: %s", strerror(errno));
                break;
            }
        }

        // 断开连接
        bIsRecvDataFault = true;

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
        zlog_error(pCategory, "Failed to close server_fd");
        exit(EXIT_FAILURE);
    }
}

static void SKA_ReceiveFromPlcThread_Udp()
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
    stServerAddress.sin_port = htons(stConfiguration.stCommConfig.ASC_RecvPortFromPlc);

    // 解析服务器IP后两位
    uint16_t serverIpLow;
    {
        unsigned int ip1, ip2, ip3, ip4;
        sscanf(stConfiguration.stCommConfig.ASC_ControlNetPortIp, "%u.%u.%u.%u", &ip1, &ip2, &ip3, &ip4);
        serverIpLow = (ip3 << 8) | ip4; // 组合 50.15 -> 0x3215
    }

    // 绑定
    if (bind(nServerFd, (struct sockaddr *)&stServerAddress, sizeof(stServerAddress)) < 0)
    {
        zlog_error(pCategory, "Bind failed");
        exit(EXIT_FAILURE);
    }

    zlog_info(
        pCategory, 
        "Recv_PLC_Data UDP Server is ready to receive on: %s:%d", 
        inet_ntoa(stServerAddress.sin_addr), ntohs(stServerAddress.sin_port)
    );

    while (1)
    {
        int nAddrLen = sizeof(stClientAddress);
        int nBytesReceived = recvfrom(nServerFd, (void *)&stComAscInputBuffer, sizeof(stComAscInputBuffer), 0, (struct sockaddr *)&stClientAddress, (socklen_t *)&nAddrLen);

        if (nBytesReceived >= 0)
        {
            // 通信正常
            bIsRecvDataFault = false;

            // zlog_debug(
            //     pCategory, 
            //     "Received %d bytes from client[%s:%d].", 
            //     nBytesReceived, inet_ntoa(stClientAddress.sin_addr), ntohs(stClientAddress.sin_port)
            // );

            // 校验防摇控制器IP后两位
            uint16_t receivedIpLow = ntohs(stComAscInputBuffer.stReceivePacketHeader.serverIpSuffix);
            if (receivedIpLow != serverIpLow)
            {
                zlog_error(
                    pCategory,
                    "IP mismatch: expected 0x%04X, received 0x%04X",
                    serverIpLow, receivedIpLow);
                continue;
            }
            // 校验报文数据长度
            uint16_t receivedDataLength = ntohs(stComAscInputBuffer.stReceivePacketHeader.dataLength);
            if (nBytesReceived != receivedDataLength)
            {
                zlog_error(
                    pCategory,
                    "Data length mismatch: expected %d, received %d",
                    receivedDataLength, nBytesReceived);
                continue;
            }

            /* 处理客户端数据 */
            SKA_ReceiveFromPlcThread_ProcessData();
        }
        else
        {
            // 通信故障
            bIsRecvDataFault = true;
            
            zlog_error(pCategory, "Failed to receive data: %s", strerror(errno));
        }
    }

    // 关闭套接字
    if (close(nServerFd) < 0)
    {
        zlog_error(pCategory, "Failed to close server_fd");
        exit(EXIT_FAILURE);
    }
}

static void SKA_ReceiveFromPlcThread_ModbusTcp()
{
    // 初始化所有寄存器为0
    uint16_t tab_registers[REGISTER_NUM];
    for (int i = 0; i < REGISTER_NUM; ++i) {
        tab_registers[i] = 0;
    }

    // 创建新的上下文
    char *strServerIp = stConfiguration.stCommConfig.ASC_ControlNetPortIp;
    uint16_t nServerPort = stConfiguration.stCommConfig.ASC_RecvPortFromPlc;
    modbus_t *pCtx = modbus_new_tcp(strServerIp, nServerPort);
    if (pCtx == NULL) {
        zlog_error(pCategory, "Failed to create the libmodbus context: %s", modbus_strerror(errno));
        exit(EXIT_FAILURE);
    }

    // 创建modbus映射
    pMbMapping = modbus_mapping_new_start_address(
        0, 0,
        0, 0,
        0, REGISTER_NUM,
        0, 0
    );
    if (pMbMapping == NULL) {
        zlog_error(pCategory, "Failed to allocate the mapping: %s", modbus_strerror(errno));
        exit(EXIT_FAILURE);
    }

    // 监听连接
    int server_socket = modbus_tcp_listen(pCtx, 1);
    if (server_socket == -1) {
        zlog_error(pCategory, "Failed to listen for connections: %s", modbus_strerror(errno));
        exit(EXIT_FAILURE);
    }

    zlog_info(pCategory, "Recv_PLC_Data Modbus TCP Server is listening on %s:%d", strServerIp, nServerPort);

    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    while (1)
    {
        // 接受连接
        if (modbus_tcp_accept(pCtx, &server_socket) == -1) {
            zlog_error(pCategory, "Failed to accept connection: %s", modbus_strerror(errno));
            continue;
        }

        // 建立连接
        bIsRecvDataFault = false;

        // 持续接收数据
        while (1)
        {
            int rc = modbus_receive(pCtx, query);
            if (rc > 0) {
                // 处理请求
                modbus_reply(pCtx, query, rc, pMbMapping);

                zlog_debug(pCategory, "Received %d bytes from client.", rc);

                SKA_ReceiveFromPlcThread_ProcessData();
            } 
            else if (rc == 0) 
            {
                // 请求被忽略
                continue;
            }
            else
            {
                // 接收请求失败
                zlog_error(pCategory, "Failed to receive data: %s", modbus_strerror(errno));
                break;
            }
        }

        // 断开连接
        bIsRecvDataFault = true;
    }

    if (close(server_socket) < 0)
    {
        zlog_error(pCategory, "Failed to close server_socket");
        exit(EXIT_FAILURE);
    }
    modbus_mapping_free(pMbMapping);
    modbus_close(pCtx);
    modbus_free(pCtx);
}

static void SKA_ReceiveFromPlcThread_ProcessData()
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

    // 接收完成时刻
    struct timespec stTime;
    if (clock_gettime(CLOCK_MONOTONIC, &stTime) != 0)
    {
        zlog_error(pCategory, "Failed to get current time");
        exit(EXIT_FAILURE);
    }
   // zlog_debug(pCategory, "Time: %ld.%09lds", stTime.tv_sec, stTime.tv_nsec);

    // 反序列化
    if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "TCP") == 0 || strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "UDP") == 0)
    {
        if (SKA_ComInt_InputN2H(&stComAscInputBuffer, &stComAscInput) != 0)
        {
            zlog_error(pCategory, "Failed to convert byte order");
            exit(EXIT_FAILURE);
        }
    }
    else if (strcmp(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "Modbus-TCP") == 0)
    {
        if (SKA_ComInt_InputDeserialize(pMbMapping->tab_registers, REGISTER_NUM, &stComAscInput) != 0)
        {
            zlog_error(pCategory, "Failed to deserialize data");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        zlog_error(pCategory, "Invalid PLC communication protocol");
        exit(EXIT_FAILURE);
    }


    // 显示输入
    // SKA_ComInt_ShowInput(&stComAscInput);
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
