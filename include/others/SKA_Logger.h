#ifndef SKA2000_OHBC_CONTROLLER_SKA_LOGGER_H
#define SKA2000_OHBC_CONTROLLER_SKA_LOGGER_H

#include <zlog.h>

// 日志分类
extern zlog_category_t *pCategory;

/**
 * @brief 日志构建
 * @return void
 */
void SKA_Logger_Create();

/**
 * @brief 日志清理
 * @return void
 */
void SKA_Logger_Destroy();

#endif // SKA2000_OHBC_CONTROLLER_SKA_LOGGER_H