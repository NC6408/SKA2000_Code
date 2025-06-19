#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "SKA_Configurer.h"
#include "SKA_Logger.h"

SKA_AppConfiguration stConfiguration;

extern char **global_argv;
extern int global_argc;

/**
 * @brief 获取字符串参数
 * @param pXpathCtx XPath上下文
 * @param strParamPath 参数路径
 * @param pValue 参数值地址
 * @return 错误码 {0: 正常}
 */
static int SKA_Configurer_GetParamString(const xmlXPathContextPtr pXpathCtx, const char *strParamPath, char *pValue);

/**
 * @brief 获取无符号16位整数参数
 * @param pXpathCtx XPath上下文
 * @param strParamPath 参数路径
 * @param pValue 参数值地址
 * @return 错误码 {0: 正常}
 */
static int SKA_Configurer_GetParamUint16(const xmlXPathContextPtr pXpathCtx, const char *strParamPath, uint16_t *pValue);

// 将IP地址字符串转换为整数
static in_addr_t ip_str_to_int(const char *ip_str);

// 将整数转换为IP地址字符串
static void int_to_ip_str(in_addr_t ip_int, char *ip_str);

// 根据IP地址和子网掩码生成网关地址
static void generate_gateway(const char *ip_str, const char *subnet_str, char *gateway_str);

static void SKA_Configurer_Create()
{
    // 初始化库并检查版本
    LIBXML_TEST_VERSION
}

static void SKA_Configurer_Destroy()
{
    // 清理资源
    xmlCleanupParser();
}

static int SKA_Configurer_ReadConfFile()
{
    SKA_Configurer_Create();

    // 解析XML文件并获取文档对象
    xmlDocPtr pDoc = xmlReadFile("./conf/app_config.xml", NULL, 0);
    if (pDoc == NULL)
    {
        zlog_error(pCategory, "'./conf/app_config.xml' not parsed successfully.");
        return -1;
    }

    // 创建XPath上下文
    xmlXPathContextPtr pXpathCtx = xmlXPathNewContext(pDoc);
    if (pXpathCtx == NULL)
    {
        zlog_error(pCategory, "Error in xmlXPathNewContext");
        xmlFreeDoc(pDoc);
        return -1;
    }

    // 获取参数

    if (SKA_Configurer_GetParamString(pXpathCtx, "/AppConfig/CommConfig/ASC_ControlNetPortIp", 
                                      stConfiguration.stCommConfig.ASC_ControlNetPortIp) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_ControlNetPortIp.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamString(pXpathCtx, "/AppConfig/CommConfig/ASC_CommProtocalWithPlc", 
                                      stConfiguration.stCommConfig.ASC_CommProtocalWithPlc) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_CommProtocalWithPlc.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamUint16(pXpathCtx, "/AppConfig/CommConfig/ASC_RecvPortFromPlc", 
                                      &stConfiguration.stCommConfig.ASC_RecvPortFromPlc) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_RecvPortFromPlc.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamUint16(pXpathCtx, "/AppConfig/CommConfig/ASC_SendPortToPlc", 
                                      &stConfiguration.stCommConfig.ASC_SendPortToPlc) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_SendPortToPlc.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamString(pXpathCtx, "/AppConfig/CommConfig/ASC_DebugNetPortIp", 
                                      stConfiguration.stCommConfig.ASC_DebugNetPortIp) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_DebugNetPortIp.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamUint16(pXpathCtx, "/AppConfig/CommConfig/ASC_DebugServerPort", 
                                      &stConfiguration.stCommConfig.ASC_DebugServerPort) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/ASC_DebugServerPort.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamString(pXpathCtx, "/AppConfig/CommConfig/PLC_ControlNetPortIp", 
                                      stConfiguration.stCommConfig.PLC_ControlNetPortIp) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/PLC_ControlNetPortIp.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }
    if (SKA_Configurer_GetParamUint16(pXpathCtx, "/AppConfig/CommConfig/PLC_RecvPortFromAsc", 
                                      &stConfiguration.stCommConfig.PLC_RecvPortFromAsc) != 0)
    {
        zlog_error(pCategory, "Failed to get /AppConfig/CommConfig/PLC_RecvPortFromAsc.");
        xmlXPathFreeContext(pXpathCtx);
        xmlFreeDoc(pDoc);
        return -1;
    }

    // 清理资源
    xmlXPathFreeContext(pXpathCtx);
    xmlFreeDoc(pDoc);

    SKA_Configurer_Destroy();

    return 0;
}

static void SKA_Configurer_UseDefault()
{
    // 设置默认参数
    strcpy(stConfiguration.stCommConfig.ASC_ControlNetPortIp, "");
    strcpy(stConfiguration.stCommConfig.ASC_CommProtocalWithPlc, "TCP");
    stConfiguration.stCommConfig.ASC_RecvPortFromPlc = 0;
    stConfiguration.stCommConfig.ASC_SendPortToPlc = 0;
    strcpy(stConfiguration.stCommConfig.ASC_DebugNetPortIp, "");
    stConfiguration.stCommConfig.ASC_DebugServerPort = 0;
    strcpy(stConfiguration.stCommConfig.PLC_ControlNetPortIp, "");
    stConfiguration.stCommConfig.PLC_RecvPortFromAsc = 0;
}

void SKA_Configurer_Config()
{
    if (SKA_Configurer_ReadConfFile() != 0)
    {
        zlog_info(pCategory, "Use default configuration.");
        SKA_Configurer_UseDefault();
    }
}

static int SKA_Configurer_GetParamString(const xmlXPathContextPtr pXpathCtx, const char *strParamPath, char *pValue)
{
    // 执行XPath查询
    xmlXPathObjectPtr pXpathObj = xmlXPathEvalExpression((xmlChar *)strParamPath, pXpathCtx);
    if (pXpathObj == NULL)
    {
        zlog_error(pCategory, "Error in xmlXPathEvalExpression");
        return -1;
    }

    // 检查是否找到节点
    if (xmlXPathNodeSetIsEmpty(pXpathObj->nodesetval))
    {
        zlog_error(pCategory, "No matching nodes found.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }

    // 检查是否存在多个匹配节点
    if (pXpathObj->nodesetval->nodeNr > 1)
    {
        zlog_error(pCategory, "Multiple matching nodes found.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }

    // 获取标签内容
    xmlNodePtr pNode = pXpathObj->nodesetval->nodeTab[0];
    if (pNode->type != XML_ELEMENT_NODE)
    {
        zlog_error(pCategory, "Invalid pNode type.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }
    xmlChar *keyValue = xmlNodeGetContent(pNode);
    strcpy(pValue, (char *)keyValue);
    xmlFree(keyValue); // 释放分配给keyValue的内存
    xmlXPathFreeObject(pXpathObj);
    return 0;
}

static int SKA_Configurer_GetParamUint16(const xmlXPathContextPtr pXpathCtx, const char *strParamPath, uint16_t *pValue)
{
    // 执行XPath查询
    xmlXPathObjectPtr pXpathObj = xmlXPathEvalExpression((xmlChar *)strParamPath, pXpathCtx);
    if (pXpathObj == NULL)
    {
        zlog_error(pCategory, "Error in xmlXPathEvalExpression");
        return -1;
    }

    // 检查是否找到节点
    if (xmlXPathNodeSetIsEmpty(pXpathObj->nodesetval))
    {
        zlog_error(pCategory, "No matching nodes found.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }

    // 检查是否存在多个匹配节点
    if (pXpathObj->nodesetval->nodeNr > 1)
    {
        zlog_error(pCategory, "Multiple matching nodes found.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }

    // 获取标签内容
    xmlNodePtr pNode = pXpathObj->nodesetval->nodeTab[0];
    if (pNode->type != XML_ELEMENT_NODE)
    {
        zlog_error(pCategory, "Invalid pNode type.");
        xmlXPathFreeObject(pXpathObj);
        return -1;
    }
    xmlChar *keyValue = xmlNodeGetContent(pNode);
    *pValue = (uint16_t)atoi((char *)keyValue);
    xmlFree(keyValue); // 释放分配给keyValue的内存
    xmlXPathFreeObject(pXpathObj);
    return 0;
}

int8_t SKA_Configurer_Update(SKA_AppConfiguration *pNewConfig)
{
    /********************** 函数参数合法性检验 ********************/
    if (pNewConfig == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_Configurer_Update().");
        return -1;
    }

    /********************** 更新参数配置 ********************/
    
    // 初始化文档 
    xmlDocPtr doc = xmlNewDoc(BAD_CAST "1.0");
    xmlNodePtr root = xmlNewNode(NULL, BAD_CAST "AppConfig");
    xmlDocSetRootElement(doc, root);
 
    /* 添加通信配置 */
    char buffer[128];
    xmlNodePtr comm = xmlNewChild(root, NULL, BAD_CAST "CommConfig", NULL);
    // ASC控制网口IP
    xmlNewChild(comm, NULL, BAD_CAST "ASC_ControlNetPortIp", 
                BAD_CAST pNewConfig->stCommConfig.ASC_ControlNetPortIp);
    // ASC与PLC的通信协议(TCP;UDP;Modbus-TCP)
    xmlNewChild(comm, NULL, BAD_CAST "ASC_CommProtocalWithPlc", 
                BAD_CAST pNewConfig->stCommConfig.ASC_CommProtocalWithPlc);
    // ASC从PLC接收数据的端口
    snprintf(buffer, sizeof(buffer), "%d", pNewConfig->stCommConfig.ASC_RecvPortFromPlc);
    xmlNewChild(comm, NULL, BAD_CAST "ASC_RecvPortFromPlc", 
                BAD_CAST buffer);
    // ASC向PLC发送数据的端口
    snprintf(buffer, sizeof(buffer), "%d", pNewConfig->stCommConfig.ASC_SendPortToPlc);
    xmlNewChild(comm, NULL, BAD_CAST "ASC_SendPortToPlc", 
                BAD_CAST buffer);
    // ASC调试网口IP
    xmlNewChild(comm, NULL, BAD_CAST "ASC_DebugNetPortIp", 
                BAD_CAST pNewConfig->stCommConfig.ASC_DebugNetPortIp);
    // ASC调试服务器端口
    snprintf(buffer, sizeof(buffer), "%d", pNewConfig->stCommConfig.ASC_DebugServerPort);
    xmlNewChild(comm, NULL, BAD_CAST "ASC_DebugServerPort", 
                BAD_CAST buffer);
    // PLC控制网口IP(仅UDP)
    xmlNewChild(comm, NULL, BAD_CAST "PLC_ControlNetPortIp", 
                BAD_CAST pNewConfig->stCommConfig.PLC_ControlNetPortIp);
    // PLC从ASC接收数据的端口(仅UDP)
     snprintf(buffer, sizeof(buffer), "%d", pNewConfig->stCommConfig.PLC_RecvPortFromAsc);
    xmlNewChild(comm, NULL, BAD_CAST "PLC_RecvPortFromAsc", 
                BAD_CAST buffer);
 
    // 保存并清理 
    if (xmlSaveFormatFileEnc("./conf/app_config.xml",  doc, "UTF-8", 1) < 0) {
        fprintf(stderr, "Failed to save XML file\n");
    }
    xmlFreeDoc(doc);
    xmlCleanupParser();

    // 设置网卡IP
    char controlNetGateway[INET_ADDRSTRLEN];
    char debugNetGateway[INET_ADDRSTRLEN];
    generate_gateway(pNewConfig->stCommConfig.ASC_ControlNetPortIp, "255.255.255.0", controlNetGateway);
    generate_gateway(pNewConfig->stCommConfig.ASC_DebugNetPortIp, "255.255.255.0", debugNetGateway);
    char config_content[1024];
    snprintf(config_content, sizeof(config_content),
             "auto lo\n"
             "iface lo inet loopback\n"
             "\n"
             "auto %s\n"
             "iface %s inet static\n"
             "    address %s\n"
             "    netmask 255.255.255.0\n"
             "    gateway %s\n"
             "    dns-nameservers 8.8.8.8 8.8.4.4"
             "\n"
             "auto %s\n"
             "iface %s inet static\n"
             "    address %s\n"
             "    netmask 255.255.255.0\n"
             "    gateway %s\n"
             "    dns-nameservers 8.8.8.8 8.8.4.4",
             "eth1", "eth1",
             pNewConfig->stCommConfig.ASC_ControlNetPortIp, 
             controlNetGateway,
             "eth0", "eth0",
             pNewConfig->stCommConfig.ASC_DebugNetPortIp,
             debugNetGateway);
    FILE *fp = fopen("/etc/network/interfaces",  "w");
    if (!fp) {
        perror("Failed to open config file");
        return 1;
    }
    fprintf(fp, "%s", config_content);
    fclose(fp);
    if (system("sudo reboot") < 0)
    {
        zlog_error(pCategory, "Cannot reboot.");
    }
    
    return 0;
}



// 将IP地址字符串转换为整数
static in_addr_t ip_str_to_int(const char *ip_str) {
    struct in_addr addr;
    inet_pton(AF_INET, ip_str, &addr);
    return addr.s_addr;
}

// 将整数转换为IP地址字符串
static void int_to_ip_str(in_addr_t ip_int, char *ip_str) {
    struct in_addr addr;
    addr.s_addr = ip_int;
    inet_ntop(AF_INET, &addr, ip_str, INET_ADDRSTRLEN);
}

// 根据IP地址和子网掩码生成网关地址
static void generate_gateway(const char *ip_str, const char *subnet_str, char *gateway_str) {
    in_addr_t ip_int = ip_str_to_int(ip_str);
    in_addr_t subnet_int = ip_str_to_int(subnet_str);

    // 计算网络地址
    in_addr_t network_int = ip_int & subnet_int;

    // 假设网关是网络地址 + 1
    in_addr_t gateway_int = network_int + htonl(1);

    // 转换回字符串格式
    int_to_ip_str(gateway_int, gateway_str);
}