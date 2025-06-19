#ifndef SKA2000_OHBC_CONTROLLER_SKA_SENDTOPLCTHREAD_H
#define SKA2000_OHBC_CONTROLLER_SKA_SENDTOPLCTHREAD_H

/**
 * @brief 向PLC发送数据的线程
 * @param arg 传入参数
 * @return void* 返回值
 */
void* SKA_SendToPlcThread_Run(void* arg);

#endif // SKA2000_OHBC_CONTROLLER_SKA_SENDTOPLCTHREAD_H