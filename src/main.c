#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

#include "main.h"
#include "SKA_Version.h"
#include "SKA_Logger.h"
#include "SKA_Configurer.h"
#include "SKA_ThreadSafety.h"
#include "SKA_CommunicationInterface.h"
#include "SKA_ReceiveFromPlcThread.h"
#include "SKA_SendToPlcThread.h"
#include "SKA_ControlThread.h"
#include "SKA_DebugServerThread.h"

/**
 * @brief 系统环境初始化
 * @return void
 */
static void SystemEnvCreate();

/**
 * @brief 系统环境清理
 * @return void
 */
static void SystemEnvDestroy();

int global_argc;
char **global_argv;

int main(int argc, char *argv[]) {
    global_argc = argc;
    global_argv = argv;

    // 系统初始化
    SystemEnvCreate();
    zlog_info(pCategory, "System initialized, version: [%s]", SKA2000_OHBC_CONTROLLER_VERSION);

    // 创建接收PLC数据线程
    pthread_t nReceivePlcDataThreadId;
    if (pthread_create(&nReceivePlcDataThreadId, NULL, SKA_ReceiveFromPlcThread_Run, NULL) != 0) {
        zlog_error(pCategory, "Failed to create ReceiveFromPlcThread");
        exit(EXIT_FAILURE);
    }

    // 创建发送PLC数据线程
    pthread_t nSendPlcDataThreadId;
    if (pthread_create(&nSendPlcDataThreadId, NULL, SKA_SendToPlcThread_Run, NULL) != 0) {
        zlog_error(pCategory, "Failed to create SendToPlcThread");
        exit(EXIT_FAILURE);
    }

    // 创建控制线程
    pthread_t nControlThreadId;
    if (pthread_create(&nControlThreadId, NULL, SKA_ControlThread_Run, NULL) != 0) {
        zlog_error(pCategory, "Failed to create ControlThread");
        exit(EXIT_FAILURE);
    }

    // 创建调试服务器线程
    pthread_t nDebugServerThreadId;
    if (pthread_create(&nDebugServerThreadId, NULL, SKA_DebugServerThread_Run, NULL) != 0) {
        zlog_error(pCategory, "Failed to create DebugServerThread");
        exit(EXIT_FAILURE);
    }

    // 等待所有线程结束
    pthread_join(nReceivePlcDataThreadId, NULL);
    pthread_join(nSendPlcDataThreadId, NULL);
    pthread_join(nControlThreadId, NULL);
    pthread_join(nDebugServerThreadId, NULL);
    zlog_info(pCategory, "All threads are terminated.");

    // 系统清理
    SystemEnvDestroy();

    return 0;
}

static void SystemEnvCreate() {
    // 忽略SIGPIPE信号, 防止进程在向已关闭的套接字发送数据时被终止
    signal(SIGPIPE, SIG_IGN);

    // 初始化日志
    SKA_Logger_Create();

    // 初始化PLC数据互斥锁
    if (pthread_mutex_init(&unPlcDataMutex, NULL) != 0) {
        zlog_error(pCategory, "Failed to initialize plc_data_mutex");
        exit(EXIT_FAILURE);
    }

    // 获取配置参数
    SKA_Configurer_Config();

    // 输入数据初始化
    if (SKA_ComInt_InitInput(&stComAscInput) != 0) {
        zlog_error(pCategory, "Failed to initialize input data");
        exit(EXIT_FAILURE);
    }

    // 输出数据初始化
    if (SKA_ComInt_InitOutput(&stComAscOutput) != 0) {
        zlog_error(pCategory, "Failed to initialize output data");
        exit(EXIT_FAILURE);
    }

    // 防摇参数初始化
    if (SKA_ComInt_InitParam(&stComAscParam) != 0) {
        zlog_error(pCategory, "Failed to initialize parameter data");
        exit(EXIT_FAILURE);
    }

    // 通信故障标志
    bIsRecvDataFault = true;
    bIsSendDataFault = true;
}

static void SystemEnvDestroy()
{
    // 销毁PLC数据互斥锁
    if (pthread_mutex_destroy(&unPlcDataMutex) != 0)
    {
        fprintf(stderr, "Failed to destroy plc_data_mutex");
        exit(EXIT_FAILURE);
    }

    // 关闭日志
    SKA_Logger_Destroy();
}