#ifndef SKA2000_OHBC_CONTROLLER_SKA_RECEIVEFROMPLCTHREAD_H
#define SKA2000_OHBC_CONTROLLER_SKA_RECEIVEFROMPLCTHREAD_H

/**
 * @brief 从PLC接收数据的线程
 * @param arg 传入参数
 * @return void* 返回值
 */
void *SKA_ReceiveFromPlcThread_Run(void *arg);

#endif // SKA2000_OHBC_CONTROLLER_SKA_RECEIVEFROMPLCTHREAD_H