#include "mongoose.h"
#include <cJSON.h>

#include "SKA_DebugServerThread.h"
#include "SKA_Logger.h"
#include "SKA_Configurer.h"
#include "SKA_CommunicationInterface.h"
#include "SKA_AntiswayController.h"

// 参数集数量
#define PARAM_SET_COUNT 8

// 参数集
SKA_ComAntiswayControllerParam g_paramSets[PARAM_SET_COUNT]; // 全局参数集
SKA_AntiswayController pASC;
// 读取json文件到参数集中
static int SKA_DebugServerTHread_LoadParamsFromJsonFile();

// 将参数集保存为JSON文件
static void SKA_DebugServerTHread_SaveParamsToJsonFile();

// 初始化参数集
static void SKA_DebugServerTHread_InitializeDefaultParams();

// 将参数集从源索引复制到目标索引
static int SKA_DebugServerThread_CopyParamSet(int sourceIndex, int destIndex);

// 事件处理函数
static void SKA_DebugServerTHread_MgEventHandler(struct mg_connection *nc, int ev, void *ev_data);

void *SKA_DebugServerThread_Run(void *arg)
{
    // 加载参数集
    if (SKA_DebugServerTHread_LoadParamsFromJsonFile() != 0)
    {
        SKA_DebugServerTHread_InitializeDefaultParams();
    }

    // 初始化事件管理器
    struct mg_mgr stMgr;
    mg_mgr_init(&stMgr);

    // 创建HTTP监听
    char *strIp = stConfiguration.stCommConfig.ASC_DebugNetPortIp;
    int nport = stConfiguration.stCommConfig.ASC_DebugServerPort;
    char strUrl[100];
    sprintf(strUrl, "http://%s:%d", strIp, nport);
    mg_http_listen(&stMgr, strUrl, SKA_DebugServerTHread_MgEventHandler, NULL);
    zlog_info(pCategory, "Debug server started on %s", strUrl);

    // 事件循环
    int nTimeoutMs = 1000;
    while (1)
    {
        mg_mgr_poll(&stMgr, nTimeoutMs);
    }

    // 释放事件管理器
    mg_mgr_free(&stMgr);
    return NULL;
}

// 读取json文件到参数集中
// 读取每个参数集的JSON文件到内存中
static int SKA_DebugServerTHread_LoadParamsFromJsonFile()
{
    for (int i = 0; i < PARAM_SET_COUNT; i++)
    {
        // 创建文件名，如 "param_set_0.json"
        char fileName[128];
        snprintf(fileName, sizeof(fileName), "./param_set_%d.json", i);

        // 打开JSON文件
        FILE *file = fopen(fileName, "r");
        if (!file)
        {
            // 如果无法打开文件，初始化默认参数
            zlog_error(pCategory, "Failed to open JSON file %s, initializing default params.", fileName);
            return -1;
        }

        // 获取文件大小
        fseek(file, 0, SEEK_END);
        long fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);

        // 分配内存
        char *buffer = (char *)malloc(fileSize + 1);
        if (!buffer)
        {
            fclose(file);
            zlog_error(pCategory, "Memory allocation failed.");
            return -1;
        }

        // 读取文件内容
        fread(buffer, 1, fileSize, file);
        fclose(file);
        buffer[fileSize] = '\0';

        // 解析JSON文件
        cJSON *json = cJSON_Parse(buffer);
        free(buffer);
        if (!json)
        {
            zlog_error(pCategory, "Failed to parse JSON file %s.", fileName);
            return -1;
        }

        // 解析参数集
        if (SKA_ComInt_ParseParamFromJson(json, &g_paramSets[i]) != 0)
        {
            zlog_error(pCategory, "Failed to parse param set %d from file %s", i, fileName);
        }

        cJSON_Delete(json);
    }
    return 0;
}

// 将参数集保存为JSON文件
static void SKA_DebugServerTHread_SaveParamsToJsonFile()
{
    // 遍历参数集
    for (int i = 0; i < PARAM_SET_COUNT; i++)
    {
        // 创建一个JSON对象
        cJSON *paramJson = NULL;
        if (SKA_ComInt_CreateParamJson(&g_paramSets[i], &paramJson) != 0)
        {
            zlog_error(pCategory, "Failed to create JSON object for param set %d", i);
            continue;
        }

        // 将JSON对象转换为字符串
        char *jsonString = cJSON_Print(paramJson);
        cJSON_Delete(paramJson);
        if (!jsonString)
        {
            zlog_error(pCategory, "Failed to convert param set %d to JSON string", i);
            continue;
        }

        // 创建文件名，如 "param_set_0.json"
        char fileName[128];
        snprintf(fileName, sizeof(fileName), "./param_set_%d.json", i);

        // 打开文件并写入数据
        FILE *file = fopen(fileName, "w");
        if (file)
        {
            fwrite(jsonString, 1, strlen(jsonString), file);
            fclose(file);
            zlog_info(pCategory, "Saved param set %d to %s", i, fileName);
        }
        else
        {
            zlog_error(pCategory, "Failed to open file %s for saving", fileName);
        }

        free(jsonString);
    }
}

// 初始化默认参数集
static void SKA_DebugServerTHread_InitializeDefaultParams()
{
    // 遍历参数集数量
    for (int i = 0; i < PARAM_SET_COUNT; i++)
    {
        // 初始化参数集
        SKA_ComInt_InitParam(&g_paramSets[i]);
    }
    // 保存参数到JSON文件
    SKA_DebugServerTHread_SaveParamsToJsonFile();
}

// 复制参数集功能
static int SKA_DebugServerThread_CopyParamSet(int sourceIndex, int destIndex)
{
    if (sourceIndex < 0 || sourceIndex >= PARAM_SET_COUNT || destIndex < 0 || destIndex >= PARAM_SET_COUNT)
    {
        zlog_error(pCategory, "Invalid parameter set index: source=%d, dest=%d", sourceIndex, destIndex);
        return -1; // 参数索引无效
    }

    // 复制内存中的参数集
    // memcpy(&g_paramSets[destIndex], &g_paramSets[sourceIndex], sizeof(SKA_ComAntiswayControllerParam));
    memcpy((void *)&g_paramSets[destIndex], (const void *)&g_paramSets[sourceIndex], sizeof(SKA_ComAntiswayControllerParam));
    zlog_info(pCategory, "Parameter set %d copied to parameter set %d", sourceIndex, destIndex);

    // 保存更新后的参数集到外存（JSON 文件）
    char fileName[128];
    snprintf(fileName, sizeof(fileName), "./param_set_%d.json", destIndex);

    // 创建一个JSON对象并保存
    cJSON *paramJson = NULL;
    if (SKA_ComInt_CreateParamJson(&g_paramSets[destIndex], &paramJson) != 0)
    {
        zlog_error(pCategory, "Failed to create JSON object for param set %d", destIndex);
        return -1;
    }

    // 将JSON对象转换为字符串
    char *jsonString = cJSON_Print(paramJson);
    cJSON_Delete(paramJson);
    if (!jsonString)
    {
        zlog_error(pCategory, "Failed to convert param set %d to JSON string", destIndex);
        return -1;
    }

    // 打开文件并保存
    FILE *file = fopen(fileName, "w");
    if (file)
    {
        fwrite(jsonString, 1, strlen(jsonString), file);
        fclose(file);
        zlog_info(pCategory, "Saved param set %d to %s", destIndex, fileName);
    }
    else
    {
        zlog_error(pCategory, "Failed to open file %s for saving", fileName);
        free(jsonString);
        return -1;
    }

    free(jsonString); // 释放内存
    return 0;         // 成功复制和保存
}

static void SKA_DebugServerTHread_MgEventHandler(struct mg_connection *nc, int ev, void *ev_data)
{
    // 判断事件类型
    if (ev == MG_EV_HTTP_MSG)
    {
        // 获取 HTTP 请求
        struct mg_http_message *pReq = (struct mg_http_message *)ev_data;
        zlog_debug(pCategory, "HTTP request: %.*s %.*s", (int)pReq->method.len, pReq->method.buf, (int)pReq->uri.len, pReq->uri.buf);
        int sourceIndex, destIndex;
        char uri[128] = {0}; // 确保 URI 以 NULL 结尾
        snprintf(uri, sizeof(uri), "%.*s", (int)pReq->uri.len, pReq->uri.buf);
        zlog_debug(pCategory, "Received URI: %s", uri);

        // 处理HTTP请求
        if (mg_match(pReq->uri, mg_str("/antiswayParams"), NULL))
        {
            if (mg_strcasecmp(pReq->method, mg_str("GET")) == 0)
            {
                // 获取请求的参数集索引
                char buf[16];
                int index;
                if (mg_http_get_var(&pReq->query, "index", buf, sizeof(buf)) > 0)
                {
                    index = atoi(buf);
                }
                else
                {
                    zlog_error(pCategory, "Failed to get parameter set index");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                // 获取参数集
                cJSON *pResJson = cJSON_CreateObject();
                cJSON_AddNumberToObject(pResJson, "code", 200);
                cJSON *pDataJson = NULL;
                if (SKA_ComInt_CreateParamJson(&g_paramSets[index], &pDataJson) != 0)
                {
                    zlog_error(pCategory, "Failed to create JSON object");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                cJSON_AddItemToObject(pResJson, "data", pDataJson);
                cJSON_AddStringToObject(pResJson, "msg", "success");
                char *strResBody = cJSON_Print(pResJson);
                if (strResBody == NULL)
                {
                    zlog_error(pCategory, "Failed to convert JSON object to string");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                // 打印响应
                // zlog_info(pCategory, "Response: %s", strResBody);
                mg_http_reply(nc, 200, "Access-Control-Allow-Origin: *\r\nContent-Type: application/json\r\n", strResBody);
                free(strResBody);
                cJSON_Delete(pResJson);
            }
            else if (mg_strcasecmp(pReq->method, mg_str("PUT")) == 0)
            {
                // 获取请求的参数集索引
                char buf[16];
                int index;
                if (mg_http_get_var(&pReq->query, "index", buf, sizeof(buf)) > 0)
                {
                    index = atoi(buf);
                }
                else
                {
                    zlog_error(pCategory, "Failed to get parameter set index");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }

                // 判断防摇控制器工作状态是否为空闲状态
                zlog_info(pCategory, "Controller state: %d", pASC.eTaskMode);
                if (pASC.eTaskMode != SKA_ASC_FREE_TASK_STATE)
                {
                    zlog_error(pCategory, "Cannot modify parameters. Controller is not FREE.");
                    mg_http_reply(nc, 503, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }

                // 更新指定的参数集
                cJSON *pReqBody = cJSON_Parse(pReq->body.buf);
                if (pReqBody == NULL)
                {
                    zlog_error(pCategory, "Failed to parse request body");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                if (pReqBody && SKA_ComInt_ParseParamFromJson(pReqBody, &g_paramSets[index]) == 0)
                {
                    SKA_DebugServerTHread_SaveParamsToJsonFile();

                    // 返回响应
                    cJSON *pResJson = cJSON_CreateObject();
                    cJSON_AddNumberToObject(pResJson, "code", 200);
                    cJSON_AddItemToObject(pResJson, "data", cJSON_CreateObject());
                    cJSON_AddStringToObject(pResJson, "msg", "success");
                    char *strResBody = cJSON_Print(pResJson);
                    mg_http_reply(nc, 200, "Access-Control-Allow-Origin: *\r\nContent-Type: application/json\r\n", strResBody);
                    free(strResBody);
                    cJSON_Delete(pResJson);
                }
                else
                {
                    zlog_error(pCategory, "Failed to parse JSON object");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                }
                cJSON_Delete(pReqBody);
            }
            else if (mg_strcasecmp(pReq->method, mg_str("OPTIONS")) == 0)
            {
                char strHeader[256];
                snprintf(strHeader, 256,
                         "%s\r\n%s\r\n%s\r\n",
                         "Access-Control-Allow-Origin: *",
                         "Access-Control-Allow-Methods: GET, PUT, OPTIONS",
                         "Access-Control-Allow-Headers: Content-Type");
                mg_http_reply(nc, 200, strHeader, "\n");
            }
            else
            {
                mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
            }

        }
        else if (mg_match(pReq->uri, mg_str("/commParams"), NULL))
        {
            if (mg_strcasecmp(pReq->method, mg_str("GET")) == 0)
            {
                cJSON *pResJson = cJSON_CreateObject();
                cJSON_AddNumberToObject(pResJson, "code", 200);

                // 获取通信参数
                cJSON *pDataJson = cJSON_CreateObject();
                cJSON_AddStringToObject(pDataJson, "ASC_ControlNetPortIp", stConfiguration.stCommConfig.ASC_ControlNetPortIp);
                cJSON_AddStringToObject(pDataJson, "ASC_CommProtocalWithPlc", stConfiguration.stCommConfig.ASC_CommProtocalWithPlc);
                cJSON_AddNumberToObject(pDataJson, "ASC_RecvPortFromPlc", stConfiguration.stCommConfig.ASC_RecvPortFromPlc);
                cJSON_AddNumberToObject(pDataJson, "ASC_SendPortToPlc", stConfiguration.stCommConfig.ASC_SendPortToPlc);
                cJSON_AddStringToObject(pDataJson, "ASC_DebugNetPortIp", stConfiguration.stCommConfig.ASC_DebugNetPortIp);
                cJSON_AddNumberToObject(pDataJson, "ASC_DebugServerPort", stConfiguration.stCommConfig.ASC_DebugServerPort);
                cJSON_AddStringToObject(pDataJson, "PLC_ControlNetPortIp", stConfiguration.stCommConfig.PLC_ControlNetPortIp);
                cJSON_AddNumberToObject(pDataJson, "PLC_RecvPortFromAsc", stConfiguration.stCommConfig.PLC_RecvPortFromAsc);

                cJSON_AddItemToObject(pResJson, "data", pDataJson);
                cJSON_AddStringToObject(pResJson, "msg", "success");
                char *strResBody = cJSON_Print(pResJson);
                if (strResBody == NULL)
                {
                    zlog_error(pCategory, "Failed to convert JSON object to string");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                zlog_info(pCategory, "Response: %s", strResBody);
                mg_http_reply(nc, 200, "Access-Control-Allow-Origin: *\r\nContent-Type: application/json\r\n", strResBody);
                free(strResBody);
                cJSON_Delete(pResJson);
            }
            else if (mg_strcasecmp(pReq->method, mg_str("PUT")) == 0)
            {
                // 获取请求体
                cJSON *pReqBody = cJSON_Parse(pReq->body.buf);
                if (pReqBody == NULL)
                {
                    zlog_error(pCategory, "Failed to parse request body");
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                SKA_AppConfiguration stNewConfig = stConfiguration;
                cJSON *pJsonTmp = NULL;
                pJsonTmp = cJSON_GetObjectItem(pReqBody, "ASC_ControlNetPortIp");
                if (pJsonTmp == NULL)
                {
                    zlog_error(pCategory, "Failed to get ASC_ControlNetPortIp from JSON object.");
                    cJSON_Delete(pReqBody);
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                strncpy(stNewConfig.stCommConfig.ASC_ControlNetPortIp, pJsonTmp->valuestring, sizeof(stNewConfig.stCommConfig.ASC_ControlNetPortIp));
                pJsonTmp = cJSON_GetObjectItem(pReqBody, "ASC_CommProtocalWithPlc");
                if (pJsonTmp == NULL)
                {
                    zlog_error(pCategory, "Failed to get ASC_CommProtocalWithPlc from JSON object.");
                    cJSON_Delete(pReqBody);
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                strncpy(stNewConfig.stCommConfig.ASC_CommProtocalWithPlc, pJsonTmp->valuestring, sizeof(stNewConfig.stCommConfig.ASC_CommProtocalWithPlc));
                pJsonTmp = cJSON_GetObjectItem(pReqBody, "ASC_DebugNetPortIp");
                if (pJsonTmp == NULL)
                {
                    zlog_error(pCategory, "Failed to get ASC_DebugNetPortIp from JSON object.");
                    cJSON_Delete(pReqBody);
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                strncpy(stNewConfig.stCommConfig.ASC_DebugNetPortIp, pJsonTmp->valuestring, sizeof(stNewConfig.stCommConfig.ASC_DebugNetPortIp));
                pJsonTmp = cJSON_GetObjectItem(pReqBody, "PLC_ControlNetPortIp");
                if (pJsonTmp == NULL)
                {
                    zlog_error(pCategory, "Failed to get PLC_ControlNetPortIp from JSON object.");
                    cJSON_Delete(pReqBody);
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                strncpy(stNewConfig.stCommConfig.PLC_ControlNetPortIp, pJsonTmp->valuestring, sizeof(stNewConfig.stCommConfig.PLC_ControlNetPortIp));
                pJsonTmp = cJSON_GetObjectItem(pReqBody, "PLC_RecvPortFromAsc");
                if (pJsonTmp == NULL)
                {
                    zlog_error(pCategory, "Failed to get PLC_RecvPortFromAsc from JSON object.");
                    cJSON_Delete(pReqBody);
                    mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
                    return;
                }
                stNewConfig.stCommConfig.PLC_RecvPortFromAsc = pJsonTmp->valueint;
                cJSON_Delete(pReqBody);
                
                // 返回响应
                cJSON *pResJson = cJSON_CreateObject();
                cJSON_AddNumberToObject(pResJson, "code", 200);
                cJSON_AddItemToObject(pResJson, "data", cJSON_CreateObject());
                cJSON_AddStringToObject(pResJson, "msg", "success");
                char *strResBody = cJSON_Print(pResJson);
                mg_http_reply(nc, 200, "Access-Control-Allow-Origin: *\r\nContent-Type: application/json\r\n", strResBody);
                free(strResBody);
                cJSON_Delete(pResJson);

                // 持久化更新通信参数
                SKA_Configurer_Update(&stNewConfig);
            }
            else if (mg_strcasecmp(pReq->method, mg_str("OPTIONS")) == 0)
            {
                char strHeader[256];
                snprintf(strHeader, 256,
                         "%s\r\n%s\r\n%s\r\n",
                         "Access-Control-Allow-Origin: *",
                         "Access-Control-Allow-Methods: GET, PUT, OPTIONS",
                         "Access-Control-Allow-Headers: Content-Type");
                mg_http_reply(nc, 200, strHeader, "\n");
            }
            else
            {
                mg_http_reply(nc, 500, "Access-Control-Allow-Origin: *\r\n", "\n");
            }

        }
        else
        {
            mg_http_reply(nc, 404, "Access-Control-Allow-Origin: *\r\n", "\n");
        }
    }
}