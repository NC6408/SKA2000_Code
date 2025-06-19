#define _POSIX_C_SOURCE 200809L // 启用CLOCK_MONOTONIC
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "SKA_ControlThread.h"
#include "SKA_ThreadSafety.h"
#include "SKA_Logger.h"
#include "SKA_CommunicationInterface.h"
#include "SKA_AntiswayController.h"
#include "SKA_DebugServerThread.h"

#define PERIOD_S 0.1 // 控制周期

// 全局变量，参数集数量
#define PARAM_SET_COUNT 8

extern SKA_ComAntiswayControllerParam g_paramSets[PARAM_SET_COUNT];

static SKA_AntiswayController stAntiswayController;

/**
 * @brief 复位防摇控制器
 * @return void
 */
static void SKA_ControlThread_Reset();

void* SKA_ControlThread_Run(void* arg) {
    // 初始化防摇控制器
    SKA_ASC_Init(&stAntiswayController, PERIOD_S);

    // 获取当前时间
    struct timespec stNext;
    // CLOCK_MONOTONIC时钟用于获取系统的单调时间, 从某个固定点（通常是系统启动时间）开始的时间，不受系统时间调整的影响
    if (clock_gettime(CLOCK_MONOTONIC, &stNext) != 0)
    {
        zlog_error(pCategory, "Failed to get current time");
        exit(EXIT_FAILURE);
    }

    while (1) {
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

        // 执行控制算法
      //  zlog_debug(pCategory, "ControlThread: %ld.%09lds", stNext.tv_sec, stNext.tv_nsec);
        
        // 设置输入
        SKA_ASC_SetInput(&(stAntiswayController.stInput), &stComAscInput);
        
        // 通信断连时复位
        if (bIsRecvDataFault || bIsSendDataFault)
        {
            SKA_ControlThread_Reset();
        }

        // 设置参数
        uint8_t nselectedParamsetIndex = stComAscInput.stGeneralInput.stCtrlBits.stBits.nParamSetIndex;
        // 参数集索引反馈
         stComAscOutput.stGeneralOutput.stStateBits.stBits.nSelectedParamSetIndex = nselectedParamsetIndex;
        // 根据PLC参数集索引选择参数集
        SKA_ASC_SetParam(&(stAntiswayController.stParam), &g_paramSets[nselectedParamsetIndex]);

        // 执行控制
        SKA_ASC_Run(&stAntiswayController);

        // 获取输出
        SKA_ASC_GetOutput(&(stAntiswayController.stOutput), &stComAscOutput);

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

    return NULL;
}

static void SKA_ControlThread_Reset() {
    // 复位防摇控制器
    if (bIsRecvDataFault)
    {
        zlog_warn(pCategory, "Recv Data has Fault");
    }
    if (bIsSendDataFault)
    {
        zlog_warn(pCategory, "Send Data has Fault");
    }
    zlog_info(pCategory, "Resetting Antisway Controller");
    stAntiswayController.stInput.stGeneralInput.stCtrlBits.stBits.bReset = 1;
    stAntiswayController.stInput.stBridgeInput.stCtrlBits.stBits.bReset = 1;
    stAntiswayController.stInput.stTrolleyInput.stCtrlBits.stBits.bReset = 1;
    stAntiswayController.stInput.stHoistInput.stCtrlBits.stBits.bReset = 1;
}