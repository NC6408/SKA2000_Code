#ifndef SKA2000_OHBC_CONTROLLER_SKA_CONFIGURER_H
#define SKA2000_OHBC_CONTROLLER_SKA_CONFIGURER_H

#include <stdint.h>

/**
 * @brief 获取配置参数
 */
void SKA_Configurer_Config();

typedef struct {
    /* ASC */
    char ASC_ControlNetPortIp[16];          // 控制网口IP
    char ASC_CommProtocalWithPlc[16];       // 与PLC的通信协议(TCP;UDP;Modbus-TCP)
    uint16_t ASC_RecvPortFromPlc;           // 从PLC接收数据的端口
    uint16_t ASC_SendPortToPlc;             // 向PLC发送数据的端口
    char ASC_DebugNetPortIp[16];            // 调试网口IP
    uint16_t ASC_DebugServerPort;           // 调试服务器端口

    /* PLC */
    char PLC_ControlNetPortIp[16];          // 控制网口IP(仅UDP)
    uint16_t PLC_RecvPortFromAsc;           // 从防摇控制器接收数据的端口(仅UDP)
} SKA_CommConfig;

typedef struct {
    SKA_CommConfig stCommConfig;            // 通信配置
} SKA_AppConfiguration;

// 配置参数
extern SKA_AppConfiguration stConfiguration;

/**
 * @brief 更新配置参数
 * @param pNewConfig 新的配置参数
 * @return 错误码 {0: 正常}
 */
int8_t SKA_Configurer_Update(SKA_AppConfiguration *pNewConfig);

#endif // SKA2000_OHBC_CONTROLLER_SKA_CONFIGURER_H