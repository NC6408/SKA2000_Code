/**
 ******************************************************************************
 * @file       : SKA_CommunicationInterface.c
 * @brief      : 通信接口
 * @author     : ZhangYi
 * @version    : None
 * @date       : 2024/10/30
 ******************************************************************************
 */
//

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include "SKA_CommunicationInterface.h"
#include "SKA_Logger.h"

// 从PLC接收的数据
SKA_ComAntiswayControllerInput stComAscInput;

// 向PLC发送的数据
SKA_ComAntiswayControllerOutput stComAscOutput;

// 防摇参数配置
SKA_ComAntiswayControllerParam stComAscParam;

// 接收数据故障
bool bIsRecvDataFault;

// 发送数据故障
bool bIsSendDataFault;

int8_t SKA_ComInt_getMinValue(SKA_ComAntiswayControllerParam *pComParam)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComParam == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_getMinValue().\n");
        return -1;
    }

    /******************** 设置最小值 ********************/

    // 通用
    pComParam->stGeneralParam.nSamplePeriod = 20;
    pComParam->stGeneralParam.nMinEffRopeLength = 500;
    pComParam->stGeneralParam.nMaxEffRopeLength = 1000;
    pComParam->stGeneralParam.nTerrainInflation = -10000;

    // 大车
    pComParam->stBridgeParam.nMinPos = -1000000;
    pComParam->stBridgeParam.nMaxPos = -1000000;
    pComParam->stBridgeParam.nMaxPosErr = 5;
    pComParam->stBridgeParam.nStaticSpeed = 0;
    pComParam->stBridgeParam.nAsLevel = 0;
    pComParam->stBridgeParam.nMaxOutFreq = 0;
    pComParam->stBridgeParam.nMinMoveFreq = 0;
    pComParam->stBridgeParam.nMaxOutSpeed = 10;
    pComParam->stBridgeParam.nMaxRefSpeed = 10;
    pComParam->stBridgeParam.nMaxOutAcc = 1;
    pComParam->stBridgeParam.nMaxRefAcc = 1;
    pComParam->stBridgeParam.nMinAssistSpeed = 0;
    pComParam->stBridgeParam.nPrelimitSpeed = 0;
    pComParam->stBridgeParam.nDelayTime = 0;
    pComParam->stBridgeParam.nDriveOnDelay = 0;
    pComParam->stBridgeParam.nMaxTrackErr = 0;
    pComParam->stBridgeParam.nTrackErrLim = 0;
    pComParam->stBridgeParam.nTrackErrRateLim = 0;
    pComParam->stBridgeParam.nTrackErrFilTc = 0;
    pComParam->stBridgeParam.nPosTrackKp = 0;
    pComParam->stBridgeParam.nPosTrackKi = 0;
    pComParam->stBridgeParam.nMaxResSway = 10;
    pComParam->stBridgeParam.nAntiswayKa = 0;

    // 小车
    pComParam->stTrolleyParam.nMinPos = -1000000;
    pComParam->stTrolleyParam.nMaxPos = -1000000;
    pComParam->stTrolleyParam.nMaxPosErr = 5;
    pComParam->stTrolleyParam.nStaticSpeed = 0;
    pComParam->stTrolleyParam.nAsLevel = 0;
    pComParam->stTrolleyParam.nMaxOutFreq = 0;
    pComParam->stTrolleyParam.nMinMoveFreq = 0;
    pComParam->stTrolleyParam.nMaxOutSpeed = 10;
    pComParam->stTrolleyParam.nMaxRefSpeed = 10;
    pComParam->stTrolleyParam.nMaxOutAcc = 1;
    pComParam->stTrolleyParam.nMaxRefAcc = 1;
    pComParam->stTrolleyParam.nMinAssistSpeed = 0;
    pComParam->stTrolleyParam.nPrelimitSpeed = 0;
    pComParam->stTrolleyParam.nDelayTime = 0;
    pComParam->stTrolleyParam.nDriveOnDelay = 0;
    pComParam->stTrolleyParam.nMaxTrackErr = 0;
    pComParam->stTrolleyParam.nTrackErrLim = 0;
    pComParam->stTrolleyParam.nTrackErrRateLim = 0;
    pComParam->stTrolleyParam.nTrackErrFilTc = 0;
    pComParam->stTrolleyParam.nPosTrackKp = 0;
    pComParam->stTrolleyParam.nPosTrackKi = 0;
    pComParam->stTrolleyParam.nMaxResSway = 10;
    pComParam->stTrolleyParam.nAntiswayKa = 0;

    // 起升
    pComParam->stHoistParam.nMinRopeLen = -1000000;
    pComParam->stHoistParam.nMaxRopeLen = -1000000;
    pComParam->stHoistParam.nMaxPosErr = 5;
    pComParam->stHoistParam.nStaticSpeed = 0;
    pComParam->stHoistParam.nMaxOutFreq = 0;
    pComParam->stHoistParam.nMinMoveFreq = 0;
    pComParam->stHoistParam.nMaxOutSpeed = 10;
    pComParam->stHoistParam.nMaxRefSpeed = 10;
    pComParam->stHoistParam.nMaxOutAcc = 1;
    pComParam->stHoistParam.nMaxRefAcc = 1;
    pComParam->stHoistParam.nPrelimitSpeed = 0;
    pComParam->stHoistParam.nDelayTime = 0;
    pComParam->stHoistParam.nMaxTrackErr = 0;
    pComParam->stHoistParam.nTrackErrLim = 0;
    pComParam->stHoistParam.nTrackErrRateLim = 0;
    pComParam->stHoistParam.nTrackErrFilTc = 0;
    pComParam->stHoistParam.nPosTrackKp = 0;
    pComParam->stHoistParam.nPosTrackKi = 0;

    return 0;
}

int8_t SKA_ComInt_getMaxValue(SKA_ComAntiswayControllerParam *pComParam)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComParam == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_getMaxValue().\n");
        return -1;
    }

    /******************** 设置最大值 ********************/

    // 通用
    pComParam->stGeneralParam.nSamplePeriod = 1000;
    pComParam->stGeneralParam.nMinEffRopeLength = 100000;
    pComParam->stGeneralParam.nMaxEffRopeLength = 100000;
    pComParam->stGeneralParam.nTerrainInflation = 10000;

    // 大车
    pComParam->stBridgeParam.nMinPos = 1000000;
    pComParam->stBridgeParam.nMaxPos = 1000000;
    pComParam->stBridgeParam.nMaxPosErr = 200;
    pComParam->stBridgeParam.nStaticSpeed = 1000;
    pComParam->stBridgeParam.nAsLevel = 2;
    pComParam->stBridgeParam.nMaxOutFreq = 20000;
    pComParam->stBridgeParam.nMinMoveFreq = 20000;
    pComParam->stBridgeParam.nMaxOutSpeed = 10000;
    pComParam->stBridgeParam.nMaxRefSpeed = 10000;
    pComParam->stBridgeParam.nMaxOutAcc = 5000;
    pComParam->stBridgeParam.nMaxRefAcc = 5000;
    pComParam->stBridgeParam.nMinAssistSpeed = 10000;
    pComParam->stBridgeParam.nPrelimitSpeed = 100;
    pComParam->stBridgeParam.nDelayTime = 1000;
    pComParam->stBridgeParam.nDriveOnDelay = 1000;
    pComParam->stBridgeParam.nMaxTrackErr = 50000;
    pComParam->stBridgeParam.nTrackErrLim = 10000;
    pComParam->stBridgeParam.nTrackErrRateLim = 10000;
    pComParam->stBridgeParam.nTrackErrFilTc = 1000;
    pComParam->stBridgeParam.nPosTrackKp = 10000;
    pComParam->stBridgeParam.nPosTrackKi = 10000;
    pComParam->stBridgeParam.nMaxResSway = 500;
    pComParam->stBridgeParam.nAntiswayKa = 10000;

    // 小车
    pComParam->stTrolleyParam.nMinPos = 1000000;
    pComParam->stTrolleyParam.nMaxPos = 1000000;
    pComParam->stTrolleyParam.nMaxPosErr = 200;
    pComParam->stTrolleyParam.nStaticSpeed = 1000;
    pComParam->stTrolleyParam.nAsLevel = 2;
    pComParam->stTrolleyParam.nMaxOutFreq = 20000;
    pComParam->stTrolleyParam.nMinMoveFreq = 20000;
    pComParam->stTrolleyParam.nMaxOutSpeed = 10000;
    pComParam->stTrolleyParam.nMaxRefSpeed = 10000;
    pComParam->stTrolleyParam.nMaxOutAcc = 5000;
    pComParam->stTrolleyParam.nMaxRefAcc = 5000;
    pComParam->stTrolleyParam.nMinAssistSpeed = 10000;
    pComParam->stTrolleyParam.nPrelimitSpeed = 100;
    pComParam->stTrolleyParam.nDelayTime = 1000;
    pComParam->stTrolleyParam.nDriveOnDelay = 1000;
    pComParam->stTrolleyParam.nMaxTrackErr = 50000;
    pComParam->stTrolleyParam.nTrackErrLim = 10000;
    pComParam->stTrolleyParam.nTrackErrRateLim = 10000;
    pComParam->stTrolleyParam.nTrackErrFilTc = 1000;
    pComParam->stTrolleyParam.nPosTrackKp = 10000;
    pComParam->stTrolleyParam.nPosTrackKi = 10000;
    pComParam->stTrolleyParam.nMaxResSway = 500;
    pComParam->stTrolleyParam.nAntiswayKa = 10000;

    // 起升
    pComParam->stHoistParam.nMinRopeLen = 1000000;
    pComParam->stHoistParam.nMaxRopeLen = 1000000;
    pComParam->stHoistParam.nMaxPosErr = 200;
    pComParam->stHoistParam.nStaticSpeed = 1000;
    pComParam->stHoistParam.nMaxOutFreq = 20000;
    pComParam->stHoistParam.nMinMoveFreq = 20000;
    pComParam->stHoistParam.nMaxOutSpeed = 10000;
    pComParam->stHoistParam.nMaxRefSpeed = 10000;
    pComParam->stHoistParam.nMaxOutAcc = 5000;
    pComParam->stHoistParam.nMaxRefAcc = 5000;
    pComParam->stHoistParam.nPrelimitSpeed = 100;
    pComParam->stHoistParam.nDelayTime = 1000;
    pComParam->stHoistParam.nMaxTrackErr = 50000;
    pComParam->stHoistParam.nTrackErrLim = 10000;
    pComParam->stHoistParam.nTrackErrRateLim = 10000;
    pComParam->stHoistParam.nTrackErrFilTc = 1000;
    pComParam->stHoistParam.nPosTrackKp = 10000;
    pComParam->stHoistParam.nPosTrackKi = 10000;

    return 0;
}

int8_t SKA_ComInt_getDefaultValue(SKA_ComAntiswayControllerParam *pComParam)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComParam == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_getDefaultValue().\n");
        return -1;
    }

    /******************** 设置默认值 ********************/

    // 通用
    pComParam->stGeneralParam.nSamplePeriod = 50;
    pComParam->stGeneralParam.nMinEffRopeLength = 1000;
    pComParam->stGeneralParam.nMaxEffRopeLength = 100000;
    pComParam->stGeneralParam.nTerrainInflation = 0;

    // 大车
    pComParam->stBridgeParam.nMinPos = 0;
    pComParam->stBridgeParam.nMaxPos = 500000;
    pComParam->stBridgeParam.nMaxPosErr = 50;
    pComParam->stBridgeParam.nStaticSpeed = 15;
    pComParam->stBridgeParam.nAsLevel = 1;
    pComParam->stBridgeParam.nMaxOutFreq = 5000;
    pComParam->stBridgeParam.nMinMoveFreq = 50;
    pComParam->stBridgeParam.nMaxOutSpeed = 500;
    pComParam->stBridgeParam.nMaxRefSpeed = 450;
    pComParam->stBridgeParam.nMaxOutAcc = 250;
    pComParam->stBridgeParam.nMaxRefAcc = 200;
    pComParam->stBridgeParam.nMinAssistSpeed = 0;
    pComParam->stBridgeParam.nPrelimitSpeed = 10;
    pComParam->stBridgeParam.nDelayTime = 150;
    pComParam->stBridgeParam.nDriveOnDelay = 0;
    pComParam->stBridgeParam.nMaxTrackErr = 1000;
    pComParam->stBridgeParam.nTrackErrLim = 200;
    pComParam->stBridgeParam.nTrackErrRateLim = 100;
    pComParam->stBridgeParam.nTrackErrFilTc = 200;
    pComParam->stBridgeParam.nPosTrackKp = 500;
    pComParam->stBridgeParam.nPosTrackKi = 0;
    pComParam->stBridgeParam.nMaxResSway = 50;
    pComParam->stBridgeParam.nAntiswayKa = 100;

    // 小车
    pComParam->stTrolleyParam.nMinPos = 0;
    pComParam->stTrolleyParam.nMaxPos = 500000;
    pComParam->stTrolleyParam.nMaxPosErr = 50;
    pComParam->stTrolleyParam.nStaticSpeed = 15;
    pComParam->stTrolleyParam.nAsLevel = 1;
    pComParam->stTrolleyParam.nMaxOutFreq = 5000;
    pComParam->stTrolleyParam.nMinMoveFreq = 50;
    pComParam->stTrolleyParam.nMaxOutSpeed = 500;
    pComParam->stTrolleyParam.nMaxRefSpeed = 450;
    pComParam->stTrolleyParam.nMaxOutAcc = 250;
    pComParam->stTrolleyParam.nMaxRefAcc = 200;
    pComParam->stTrolleyParam.nMinAssistSpeed = 0;
    pComParam->stTrolleyParam.nPrelimitSpeed = 10;
    pComParam->stTrolleyParam.nDelayTime = 150;
    pComParam->stTrolleyParam.nDriveOnDelay = 0;
    pComParam->stTrolleyParam.nMaxTrackErr = 1000;
    pComParam->stTrolleyParam.nTrackErrLim = 200;
    pComParam->stTrolleyParam.nTrackErrRateLim = 100;
    pComParam->stTrolleyParam.nTrackErrFilTc = 200;
    pComParam->stTrolleyParam.nPosTrackKp = 500;
    pComParam->stTrolleyParam.nPosTrackKi = 0;
    pComParam->stTrolleyParam.nMaxResSway = 50;
    pComParam->stTrolleyParam.nAntiswayKa = 100;

    // 起升
    pComParam->stHoistParam.nMinRopeLen = 0;
    pComParam->stHoistParam.nMaxRopeLen = 500000;
    pComParam->stHoistParam.nMaxPosErr = 50;
    pComParam->stHoistParam.nStaticSpeed = 15;
    pComParam->stHoistParam.nMaxOutFreq = 5000;
    pComParam->stHoistParam.nMinMoveFreq = 50;
    pComParam->stHoistParam.nMaxOutSpeed = 500;
    pComParam->stHoistParam.nMaxRefSpeed = 450;
    pComParam->stHoistParam.nMaxOutAcc = 250;
    pComParam->stHoistParam.nMaxRefAcc = 200;
    pComParam->stHoistParam.nPrelimitSpeed = 10;
    pComParam->stHoistParam.nDelayTime = 150;
    pComParam->stHoistParam.nMaxTrackErr = 1000;
    pComParam->stHoistParam.nTrackErrLim = 200;
    pComParam->stHoistParam.nTrackErrRateLim = 100;
    pComParam->stHoistParam.nTrackErrFilTc = 200;
    pComParam->stHoistParam.nPosTrackKp = 500;
    pComParam->stHoistParam.nPosTrackKi = 0;

    return 0;
}

int8_t SKA_ComInt_InitInput(SKA_ComAntiswayControllerInput *pComInput)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComInput == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_InitInput().\n");
        return -1;
    }

    /******************** 初始化输入 ********************/

    /* 一般数据初始化为0 */
    memset((void *)pComInput, 0, sizeof(SKA_ComAntiswayControllerInput));

    /* 需要上升沿检测的数据初始化为1 */

    // 通用
    pComInput->stGeneralInput.stCtrlBits.stBits.bStartVerticalTask = 1;
    pComInput->stGeneralInput.stCtrlBits.stBits.bStartHorizontalTask = 1;

    // 大车
    pComInput->stBridgeInput.stCtrlBits.stBits.bStart = 1;
    pComInput->stBridgeInput.stCtrlBits.stBits.bReset = 1;
    pComInput->stBridgeInput.stCtrlBits.stBits.bQuickStop = 1;
    pComInput->stBridgeInput.stCtrlBits.stBits.bAntiswayStop = 1;

    // 小车
    pComInput->stTrolleyInput.stCtrlBits.stBits.bStart = 1;
    pComInput->stTrolleyInput.stCtrlBits.stBits.bReset = 1;
    pComInput->stTrolleyInput.stCtrlBits.stBits.bQuickStop = 1;
    pComInput->stTrolleyInput.stCtrlBits.stBits.bAntiswayStop = 1;

    // 起升
    pComInput->stHoistInput.stCtrlBits.stBits.bStart = 1;
    pComInput->stHoistInput.stCtrlBits.stBits.bReset = 1;
    pComInput->stHoistInput.stCtrlBits.stBits.bQuickStop = 1;

    return 0;
}

int8_t SKA_ComInt_InitParam(SKA_ComAntiswayControllerParam *pComParam)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComParam == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_InitParam().\n");
        return -1;
    }

    /******************** 初始化参数 ********************/
    // 设置为默认参数
    SKA_ComInt_getDefaultValue(pComParam);

    return 0;
}

int8_t SKA_ComInt_InitOutput(SKA_ComAntiswayControllerOutput *pComOutput)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComOutput == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_InitOutput().\n");
        return -1;
    }

    /******************** 初始化输出 ********************/

    // 初始化为0
    memset((void *)pComOutput, 0, sizeof(SKA_ComAntiswayControllerOutput));

    return 0;
}

int8_t SKA_ComInt_ShowInput(const SKA_ComAntiswayControllerInput *pComAscInput)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComAscInput == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_ShowInput().\n");
        return -1;
    }

    /******************** 打印输入 ********************/
    printf("nReceiveHeader.serverIpSuffix: %d\n", pComAscInput->stReceivePacketHeader.serverIpSuffix);
    printf("nReceiveHeader.dataLength: %d\n", pComAscInput->stReceivePacketHeader.dataLength);
    printf("stGeneralInput.nPlcHeartbeat: %d\n", pComAscInput->stGeneralInput.nPlcHeartbeat);
    printf("stGeneralInput.stCtrlBits: %d\n", pComAscInput->stGeneralInput.stCtrlBits.nData);
    printf("stGeneralInput.nCurRopeLength: %d\n", pComAscInput->stGeneralInput.nCurRopeLength);

    printf("stBridgeInput.stCtrlBits: %d\n", pComAscInput->stBridgeInput.stCtrlBits.nData);
    printf("stBridgeInput.nRopeOffset: %d\n", pComAscInput->stBridgeInput.nRopeOffset);
    printf("stBridgeInput.nCurPos: %d\n", pComAscInput->stBridgeInput.nCurPos);
    printf("stBridgeInput.nTargetPos: %d\n", pComAscInput->stBridgeInput.nTargetPos);
    printf("stBridgeInput.nSpeedLimit: %d\n", pComAscInput->stBridgeInput.nSpeedLimit);
    printf("stBridgeInput.nManualFreq: %d\n", pComAscInput->stBridgeInput.nManualFreq);

    printf("stTrolleyInput.stCtrlBits: %d\n", pComAscInput->stTrolleyInput.stCtrlBits.nData);
    printf("stTrolleyInput.nRopeOffset: %d\n", pComAscInput->stTrolleyInput.nRopeOffset);
    printf("stTrolleyInput.nCurPos: %d\n", pComAscInput->stTrolleyInput.nCurPos);
    printf("stTrolleyInput.nTargetPos: %d\n", pComAscInput->stTrolleyInput.nTargetPos);
    printf("stTrolleyInput.nSpeedLimit: %d\n", pComAscInput->stTrolleyInput.nSpeedLimit);
    printf("stTrolleyInput.nManualFreq: %d\n", pComAscInput->stTrolleyInput.nManualFreq);

    printf("stHoistInput.stCtrlBits: %d\n", pComAscInput->stHoistInput.stCtrlBits.nData);
    printf("stHoistInput.nTargetRopeLength: %d\n", pComAscInput->stHoistInput.nTargetRopeLength);
    printf("stHoistInput.nSpeedLimit: %d\n", pComAscInput->stHoistInput.nSpeedLimit);
    printf("stHoistInput.nManualFreq: %d\n", pComAscInput->stHoistInput.nManualFreq);

    printf("stCameraInput.stStateBits: %d\n", pComAscInput->stCameraInput.stStateBits.nData);
    printf("stCameraInput.stErrorBits: %d\n", pComAscInput->stCameraInput.stErrorBits.nData);
    printf("stCameraInput.stWarnBits: %d\n", pComAscInput->stCameraInput.stWarnBits.nData);
    printf("stCameraInput.nSwayPositionX: %d\n", pComAscInput->stCameraInput.nSwayPositionX);
    printf("stCameraInput.nSwayVelocityX: %d\n", pComAscInput->stCameraInput.nSwayVelocityX);
    printf("stCameraInput.nSwayPositionY: %d\n", pComAscInput->stCameraInput.nSwayPositionY);
    printf("stCameraInput.nSwayVelocityY: %d\n", pComAscInput->stCameraInput.nSwayVelocityY);
    printf("stCameraInput.nRotationAngle: %d\n", pComAscInput->stCameraInput.nRotationAngle);
    printf("stCameraInput.nRotationVelocity: %d\n", pComAscInput->stCameraInput.nRotationVelocity);
    // 纵向避障
    printf("stVerticalInput.nTotalHeight: %d\n", pComAscInput->stVerticalInput.nTotalHeight);
    printf("stVerticalInput.arrLiftTerrain: (d, h)\n[");
    for (int i = 0; i < 10; i++)
    {
        printf("(%d, %d), ", pComAscInput->stVerticalInput.arrLiftTerrain[i].nDistance, pComAscInput->stVerticalInput.arrLiftTerrain[i].nHeight);
    }
    printf("]\n");
    printf("stVerticalInput.nLiftArraySize: %d\n", pComAscInput->stVerticalInput.nLiftArraySize);
    printf("stVerticalInput.arrDropTerrain: (d, h)\n[");
    for (int i = 0; i < 10; i++)
    {
        printf("(%d, %d), ", pComAscInput->stVerticalInput.arrDropTerrain[i].nDistance, pComAscInput->stVerticalInput.arrDropTerrain[i].nHeight);
    }
    printf("]\n");
    printf("stVerticalInput.nDropArraySize: %d\n", pComAscInput->stVerticalInput.nDropArraySize);

    // 横向避障
    printf("stHorizontalInput.nObstacleNum: %d\n", pComAscInput->stHorizontalInput.nObstacleNum);
    printf("stHorizontalInput.nWaypointsNum: %d\n", pComAscInput->stHorizontalInput.nWaypointsNum);
    printf("stHorizontalInput.nObstacleInflation: %d\n", pComAscInput->stHorizontalInput.nObstacleInflation);
    printf("stHorizontalInput.MidWaypointsCoordArray: (d, w)\n[");
    for (int i = 0; i < 20; i++)
    {
        printf("(%d, %d), ", pComAscInput->stHorizontalInput.arrWaypoints[i].nWaypointsCoordX, pComAscInput->stHorizontalInput.arrWaypoints[i].nWaypointsCoordY);
    }
    printf("]\n");
    printf("stHorizontalInput.ObstacleCoordArray: (X, Y, W, H)\n[");
    for (int i = 0; i < 5; i++)
    {
        printf("(%d, %d, %d, %d), ", pComAscInput->stHorizontalInput.arrObstacle[i].nObstacleCoordX, pComAscInput->stHorizontalInput.arrObstacle[i].nObstacleCoordY,
               pComAscInput->stHorizontalInput.arrObstacle[i].nObstacleCoordW, pComAscInput->stHorizontalInput.arrObstacle[i].nObstacleCoordH);
    }
    printf("]\n");

    return 0;
}

int8_t SKA_ComInt_ShowOutput(const SKA_ComAntiswayControllerOutput *pComAscOutput)
{
    /******************** 函数参数合法性检验 ********************/
    if (pComAscOutput == NULL)
    {
        printf("Invalid parameters of SKA_ComInt_ShowOutput().\n");
        return -1;
    }

    /******************** 打印输出 ********************/
    printf("nSendHeader.serverIpSuffix: %d\n", pComAscOutput->stSendPacketHeader.serverIpSuffix);
    printf("nSendHeader.dataLength: %d\n", pComAscOutput->stSendPacketHeader.dataLength);      printf("stGeneralOutput.nAscHeartbeat: %d\n", pComAscOutput->stGeneralOutput.nAscHeartbeat);
    printf("stGeneralOutput.stStateBits: %d\n", pComAscOutput->stGeneralOutput.stStateBits.nData);
    printf("stGeneralOutput.stErrorBits: %d\n", pComAscOutput->stGeneralOutput.stErrorBits.nData);
    printf("stGeneralOutput.stWarnBits: %d\n", pComAscOutput->stGeneralOutput.stWarnBits.nData);

    printf("stBridgeOutput.stStateBits: %d\n", pComAscOutput->stBridgeOutput.stStateBits.nData);
    printf("stBridgeOutput.stErrorBits: %d\n", pComAscOutput->stBridgeOutput.stErrorBits.nData);
    printf("stBridgeOutput.stWarnBits: %d\n", pComAscOutput->stBridgeOutput.stWarnBits.nData);
    printf("stBridgeOutput.nOutputFreq: %d\n", pComAscOutput->stBridgeOutput.nOutputFreq);
    printf("stBridgeOutput.nOutputSpeed: %d\n", pComAscOutput->stBridgeOutput.nOutputSpeed);
    printf("stBridgeOutput.nBrakeDist: %d\n", pComAscOutput->stBridgeOutput.nBrakeDist);
    printf("stBridgeOutput.nEffRopeLen: %d\n", pComAscOutput->stBridgeOutput.nEffRopeLen);
    printf("stBridgeOutput.nTargetPos: %d\n", pComAscOutput->stBridgeOutput.nTargetPos);
    printf("stBridgeOutput.nOriTrackErr: %d\n", pComAscOutput->stBridgeOutput.nOriTrackErr);
    printf("stBridgeOutput.nFilTrackErr: %d\n", pComAscOutput->stBridgeOutput.nFilTrackErr);

    printf("stTrolleyOutput.stStateBits: %d\n", pComAscOutput->stTrolleyOutput.stStateBits.nData);
    printf("stTrolleyOutput.stErrorBits: %d\n", pComAscOutput->stTrolleyOutput.stErrorBits.nData);
    printf("stTrolleyOutput.stWarnBits: %d\n", pComAscOutput->stTrolleyOutput.stWarnBits.nData);
    printf("stTrolleyOutput.nOutputFreq: %d\n", pComAscOutput->stTrolleyOutput.nOutputFreq);
    printf("stTrolleyOutput.nOutputSpeed: %d\n", pComAscOutput->stTrolleyOutput.nOutputSpeed);
    printf("stTrolleyOutput.nBrakeDist: %d\n", pComAscOutput->stTrolleyOutput.nBrakeDist);
    printf("stTrolleyOutput.nEffRopeLen: %d\n", pComAscOutput->stTrolleyOutput.nEffRopeLen);
    printf("stTrolleyOutput.nTargetPos: %d\n", pComAscOutput->stTrolleyOutput.nTargetPos);
    printf("stTrolleyOutput.nOriTrackErr: %d\n", pComAscOutput->stTrolleyOutput.nOriTrackErr);
    printf("stTrolleyOutput.nFilTrackErr: %d\n", pComAscOutput->stTrolleyOutput.nFilTrackErr);

    printf("stHoistOutput.stStateBits: %d\n", pComAscOutput->stHoistOutput.stStateBits.nData);
    printf("stHoistOutput.stErrorBits: %d\n", pComAscOutput->stHoistOutput.stErrorBits.nData);
    printf("stHoistOutput.stWarnBits: %d\n", pComAscOutput->stHoistOutput.stWarnBits.nData);
    printf("stHoistOutput.nOutputFreq: %d\n", pComAscOutput->stHoistOutput.nOutputFreq);
    printf("stHoistOutput.nOutputSpeed: %d\n", pComAscOutput->stHoistOutput.nOutputSpeed);
    printf("stHoistOutput.nTargetRopeLength: %d\n", pComAscOutput->stHoistOutput.nTargetRopeLength);
    printf("stHoistOutput.nOriTrackErr: %d\n", pComAscOutput->stHoistOutput.nOriTrackErr);
    printf("stHoistOutput.nFilTrackErr: %d\n", pComAscOutput->stHoistOutput.nFilTrackErr);

    return 0;
}

int8_t SKA_ComInt_InputN2H(const SKA_ComAntiswayControllerInput *pComAscInputN, SKA_ComAntiswayControllerInput *pComAscInputH)
{
    /********************** 函数参数合法性检验 ********************/
    if (pComAscInputN == NULL || pComAscInputH == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_InputN2H().");
        return -1;
    }

    /********************** 字节顺序转换 ********************/
    pComAscInputH->stReceivePacketHeader.serverIpSuffix = ntohs(pComAscInputN->stReceivePacketHeader.serverIpSuffix);
    pComAscInputH->stReceivePacketHeader.dataLength = ntohs(pComAscInputN->stReceivePacketHeader.dataLength);
    pComAscInputH->stGeneralInput.nPlcHeartbeat = ntohs(pComAscInputN->stGeneralInput.nPlcHeartbeat);
    pComAscInputH->stGeneralInput.stCtrlBits.nData = ntohs(pComAscInputN->stGeneralInput.stCtrlBits.nData);
    pComAscInputH->stGeneralInput.nCurRopeLength = (int32_t)ntohl((uint32_t)pComAscInputN->stGeneralInput.nCurRopeLength);

    pComAscInputH->stBridgeInput.stCtrlBits.nData = ntohs(pComAscInputN->stBridgeInput.stCtrlBits.nData);
    pComAscInputH->stBridgeInput.nRopeOffset = (int16_t)ntohs((uint16_t)pComAscInputN->stBridgeInput.nRopeOffset);
    pComAscInputH->stBridgeInput.nCurPos = (int32_t)ntohl((uint32_t)pComAscInputN->stBridgeInput.nCurPos);
    pComAscInputH->stBridgeInput.nTargetPos = (int32_t)ntohl((uint32_t)pComAscInputN->stBridgeInput.nTargetPos);
    pComAscInputH->stBridgeInput.nSpeedLimit = ntohs(pComAscInputN->stBridgeInput.nSpeedLimit);
    pComAscInputH->stBridgeInput.nManualFreq = (int16_t)ntohs((uint16_t)pComAscInputN->stBridgeInput.nManualFreq);

    pComAscInputH->stTrolleyInput.stCtrlBits.nData = ntohs(pComAscInputN->stTrolleyInput.stCtrlBits.nData);
    pComAscInputH->stTrolleyInput.nRopeOffset = (int16_t)ntohs((uint16_t)pComAscInputN->stTrolleyInput.nRopeOffset);
    pComAscInputH->stTrolleyInput.nCurPos = (int32_t)ntohl((uint32_t)pComAscInputN->stTrolleyInput.nCurPos);
    pComAscInputH->stTrolleyInput.nTargetPos = (int32_t)ntohl((uint32_t)pComAscInputN->stTrolleyInput.nTargetPos);
    pComAscInputH->stTrolleyInput.nSpeedLimit = ntohs(pComAscInputN->stTrolleyInput.nSpeedLimit);
    pComAscInputH->stTrolleyInput.nManualFreq = (int16_t)ntohs((uint16_t)pComAscInputN->stTrolleyInput.nManualFreq);

    pComAscInputH->stHoistInput.stCtrlBits.nData = ntohs(pComAscInputN->stHoistInput.stCtrlBits.nData);
    pComAscInputH->stHoistInput.nTargetRopeLength = (int32_t)ntohl((uint32_t)pComAscInputN->stHoistInput.nTargetRopeLength);
    pComAscInputH->stHoistInput.nSpeedLimit = ntohs(pComAscInputN->stHoistInput.nSpeedLimit);
    pComAscInputH->stHoistInput.nManualFreq = (int16_t)ntohs((uint16_t)pComAscInputN->stHoistInput.nManualFreq);

    pComAscInputH->stCameraInput.stStateBits.nData = ntohs(pComAscInputN->stCameraInput.stStateBits.nData);
    pComAscInputH->stCameraInput.stErrorBits.nData = ntohs(pComAscInputN->stCameraInput.stErrorBits.nData);
    pComAscInputH->stCameraInput.stWarnBits.nData = ntohs(pComAscInputN->stCameraInput.stWarnBits.nData);
    pComAscInputH->stCameraInput.nSwayPositionX = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nSwayPositionX);
    pComAscInputH->stCameraInput.nSwayVelocityX = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nSwayVelocityX);
    pComAscInputH->stCameraInput.nSwayPositionY = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nSwayPositionY);
    pComAscInputH->stCameraInput.nSwayVelocityY = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nSwayVelocityY);
    pComAscInputH->stCameraInput.nRotationAngle = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nRotationAngle);
    pComAscInputH->stCameraInput.nRotationVelocity = (int16_t)ntohs((uint16_t)pComAscInputN->stCameraInput.nRotationVelocity);

    pComAscInputH->stVerticalInput.nTotalHeight = (int32_t)ntohl((uint32_t)pComAscInputN->stVerticalInput.nTotalHeight);
    for (int i = 0; i < 10; i++)
    {
        pComAscInputH->stVerticalInput.arrLiftTerrain[i].nDistance = ntohs(pComAscInputN->stVerticalInput.arrLiftTerrain[i].nDistance);
        pComAscInputH->stVerticalInput.arrLiftTerrain[i].nHeight = (int16_t)ntohs((uint16_t)pComAscInputN->stVerticalInput.arrLiftTerrain[i].nHeight);
    }
    pComAscInputH->stVerticalInput.nLiftArraySize = ntohs(pComAscInputN->stVerticalInput.nLiftArraySize);
    for (int i = 0; i < 10; i++)
    {
        pComAscInputH->stVerticalInput.arrDropTerrain[i].nDistance = ntohs(pComAscInputN->stVerticalInput.arrDropTerrain[i].nDistance);
        pComAscInputH->stVerticalInput.arrDropTerrain[i].nHeight = (int16_t)ntohs((uint16_t)pComAscInputN->stVerticalInput.arrDropTerrain[i].nHeight);
    }
    pComAscInputH->stVerticalInput.nDropArraySize = ntohs(pComAscInputN->stVerticalInput.nDropArraySize);

    pComAscInputH->stHorizontalInput.nObstacleNum = ntohs(pComAscInputN->stHorizontalInput.nObstacleNum);
    pComAscInputH->stHorizontalInput.nWaypointsNum = ntohs(pComAscInputN->stHorizontalInput.nWaypointsNum);
    pComAscInputH->stHorizontalInput.nObstacleInflation = ntohs(pComAscInputN->stHorizontalInput.nObstacleInflation);   
    for (int i = 0; i < 20; i++)
    {
        pComAscInputH->stHorizontalInput.arrWaypoints[i].nWaypointsCoordX = (int32_t)ntohl((uint32_t)pComAscInputN->stHorizontalInput.arrWaypoints[i].nWaypointsCoordX);
        pComAscInputH->stHorizontalInput.arrWaypoints[i].nWaypointsCoordY = (int16_t)ntohs((uint16_t)pComAscInputN->stHorizontalInput.arrWaypoints[i].nWaypointsCoordY);
    }
        for (int i = 0; i < 5; i++)
    {
        pComAscInputH->stHorizontalInput.arrObstacle[i].nObstacleCoordX = (int32_t)ntohl((uint32_t)pComAscInputN->stHorizontalInput.arrObstacle[i].nObstacleCoordX);
        pComAscInputH->stHorizontalInput.arrObstacle[i].nObstacleCoordY = (int16_t)ntohs((uint16_t)pComAscInputN->stHorizontalInput.arrObstacle[i].nObstacleCoordY);
        pComAscInputH->stHorizontalInput.arrObstacle[i].nObstacleCoordW = (int32_t)ntohl((uint32_t)pComAscInputN->stHorizontalInput.arrObstacle[i].nObstacleCoordW);
        pComAscInputH->stHorizontalInput.arrObstacle[i].nObstacleCoordH = (int16_t)ntohs((uint16_t)pComAscInputN->stHorizontalInput.arrObstacle[i].nObstacleCoordH);
    }

    return 0;
}

int8_t SKA_ComInt_OutputH2N(const SKA_ComAntiswayControllerOutput *pComAscOutputH, SKA_ComAntiswayControllerOutput *pComAscOutputN)
{
    /************************ 函数参数合法性检验 ********************/
    if (pComAscOutputH == NULL || pComAscOutputN == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_OutputH2N().");
        return -1;
    }

    /************************ 字节顺序转换 ********************/
     pComAscOutputN->stSendPacketHeader.serverIpSuffix = htons(pComAscOutputH->stSendPacketHeader.serverIpSuffix);
     pComAscOutputN->stSendPacketHeader.dataLength = htons(pComAscOutputH->stSendPacketHeader.dataLength);
     pComAscOutputN->stGeneralOutput.nAscHeartbeat = htons(pComAscOutputH->stGeneralOutput.nAscHeartbeat);
    pComAscOutputN->stGeneralOutput.stStateBits.nData = htons(pComAscOutputH->stGeneralOutput.stStateBits.nData);
    pComAscOutputN->stGeneralOutput.stErrorBits.nData = htons(pComAscOutputH->stGeneralOutput.stErrorBits.nData);
    pComAscOutputN->stGeneralOutput.stWarnBits.nData = htons(pComAscOutputH->stGeneralOutput.stWarnBits.nData);

    pComAscOutputN->stBridgeOutput.stStateBits.nData = htons(pComAscOutputH->stBridgeOutput.stStateBits.nData);
    pComAscOutputN->stBridgeOutput.stErrorBits.nData = htons(pComAscOutputH->stBridgeOutput.stErrorBits.nData);
    pComAscOutputN->stBridgeOutput.stWarnBits.nData = htons(pComAscOutputH->stBridgeOutput.stWarnBits.nData);
    pComAscOutputN->stBridgeOutput.nOutputFreq = (int16_t)htons((uint16_t)pComAscOutputH->stBridgeOutput.nOutputFreq);
    pComAscOutputN->stBridgeOutput.nOutputSpeed = (int16_t)htons((uint16_t)pComAscOutputH->stBridgeOutput.nOutputSpeed);
    pComAscOutputN->stBridgeOutput.nBrakeDist = htons(pComAscOutputH->stBridgeOutput.nBrakeDist);
    pComAscOutputN->stBridgeOutput.nEffRopeLen = (int32_t)htonl((uint32_t)pComAscOutputH->stBridgeOutput.nEffRopeLen);
    pComAscOutputN->stBridgeOutput.nTargetPos = (int32_t)htonl((uint32_t)pComAscOutputH->stBridgeOutput.nTargetPos);
    pComAscOutputN->stBridgeOutput.nOriTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stBridgeOutput.nOriTrackErr);
    pComAscOutputN->stBridgeOutput.nFilTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stBridgeOutput.nFilTrackErr);

    pComAscOutputN->stTrolleyOutput.stStateBits.nData = htons(pComAscOutputH->stTrolleyOutput.stStateBits.nData);
    pComAscOutputN->stTrolleyOutput.stErrorBits.nData = htons(pComAscOutputH->stTrolleyOutput.stErrorBits.nData);
    pComAscOutputN->stTrolleyOutput.stWarnBits.nData = htons(pComAscOutputH->stTrolleyOutput.stWarnBits.nData);
    pComAscOutputN->stTrolleyOutput.nOutputFreq = (int16_t)htons((uint16_t)pComAscOutputH->stTrolleyOutput.nOutputFreq);
    pComAscOutputN->stTrolleyOutput.nOutputSpeed = (int16_t)htons((uint16_t)pComAscOutputH->stTrolleyOutput.nOutputSpeed);
    pComAscOutputN->stTrolleyOutput.nBrakeDist = htons(pComAscOutputH->stTrolleyOutput.nBrakeDist);
    pComAscOutputN->stTrolleyOutput.nEffRopeLen = (int32_t)htonl((uint32_t)pComAscOutputH->stTrolleyOutput.nEffRopeLen);
    pComAscOutputN->stTrolleyOutput.nTargetPos = (int32_t)htonl((uint32_t)pComAscOutputH->stTrolleyOutput.nTargetPos);
    pComAscOutputN->stTrolleyOutput.nOriTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stTrolleyOutput.nOriTrackErr);
    pComAscOutputN->stTrolleyOutput.nFilTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stTrolleyOutput.nFilTrackErr);

    pComAscOutputN->stHoistOutput.stStateBits.nData = htons(pComAscOutputH->stHoistOutput.stStateBits.nData);
    pComAscOutputN->stHoistOutput.stErrorBits.nData = htons(pComAscOutputH->stHoistOutput.stErrorBits.nData);
    pComAscOutputN->stHoistOutput.stWarnBits.nData = htons(pComAscOutputH->stHoistOutput.stWarnBits.nData);
    pComAscOutputN->stHoistOutput.nOutputFreq = (int16_t)htons((uint16_t)pComAscOutputH->stHoistOutput.nOutputFreq);
    pComAscOutputN->stHoistOutput.nOutputSpeed = (int16_t)htons((uint16_t)pComAscOutputH->stHoistOutput.nOutputSpeed);
    pComAscOutputN->stHoistOutput.nTargetRopeLength = (int32_t)htonl((uint32_t)pComAscOutputH->stHoistOutput.nTargetRopeLength);
    pComAscOutputN->stHoistOutput.nOriTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stHoistOutput.nOriTrackErr);
    pComAscOutputN->stHoistOutput.nFilTrackErr = (int32_t)htonl((uint32_t)pComAscOutputH->stHoistOutput.nFilTrackErr);

    return 0;
}

int8_t SKA_ComInt_InputDeserialize(const uint16_t *pRegisterBuffer, int32_t nRegNum, SKA_ComAntiswayControllerInput *pComAscInputH)
{
    /********************** 函数参数合法性检验 ********************/
    if (pComAscInputH == NULL || pRegisterBuffer == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_InputDeserialize().");
        return -1;
    }
    if (nRegNum * 2 < sizeof(SKA_ComAntiswayControllerInput))
    {
        zlog_error(pCategory, "Register buffer size is not enough.");
        return -1;
    }
    if (sizeof(SKA_ComAntiswayControllerInput) % 2 != 0)
    {
        zlog_error(pCategory, "Data bytes is not even.");
        return -1;
    }

    /********************** 反序列化 ********************/

    // 反序列化
    SKA_ComAntiswayControllerInput stComInputN;
    uint8_t *pByteBuffer = (uint8_t *)&stComInputN;
    int nRegisterNum = sizeof(SKA_ComAntiswayControllerInput) / 2;
    for (int i = 0; i < nRegisterNum; i++)
    {
        pByteBuffer[2 * i] = (uint8_t)(pRegisterBuffer[i] >> 8);
        pByteBuffer[2 * i + 1] = (uint8_t)(pRegisterBuffer[i] & 0xFF);
    }

    // 字节序转换
    SKA_ComInt_InputN2H(&stComInputN, pComAscInputH);

    return 0;
}

int8_t SKA_ComInt_OutputSerialize(const SKA_ComAntiswayControllerOutput *pComOutputH, uint16_t *pRegisterBuffer, int32_t nRegNum)
{
    /********************** 函数参数合法性检验 ********************/
    if (pComOutputH == NULL || pRegisterBuffer == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_OutputSerialize().");
        return -1;
    }
    if (nRegNum * 2 < sizeof(SKA_ComAntiswayControllerOutput))
    {
        zlog_error(pCategory, "Register buffer size is not enough.");
        return -1;
    }
    if (sizeof(SKA_ComAntiswayControllerOutput) % 2 != 0)
    {
        zlog_error(pCategory, "Data bytes is not even.");
        return -1;
    }

    /********************** 序列化 ********************/

    /*
        假设需要发送一个uint32_t，字面值为0x12345678
        若主机为小端系统，则内存中存储为0x78 0x56 0x34 0x12；若主机为大端系统，则内存中存储为0x12 0x34 0x56 0x78
        因为网络字节序为大端序，所以需要将内存中的数据转换为大端序再发送，即期望发送的字节流为0x12 0x34 0x56 0x78
        为了实现这个目的，可以使用htons()、htonl()等函数，将内存中的数据转换为网络字节序
        对于TCP/UDP协议，可以以字节（uint8_t）为单位处理通信数据，即直接发送已经调整好字节序的字节流（uint8_t数组）
        而对于Modbus TCP，需要以寄存器（uint16_t）为单位来处理通信数据，所以需要先将已经调整好字节序的字节流转换为寄存器数据（uint16_t数组），再发送寄存器数据
        libmodbus在发送寄存器数据时，直接对uint16_t的字面值进行处理，获取大端序的字节流，而非考虑主机字节序
        因此，若期望发送的字节流为0x12 0x34 0x56 0x78，则需要构建出的寄存器数据为{0x1234, 0x5678}
    */

    // 字节序转换
    SKA_ComAntiswayControllerOutput stComOutputN;
    SKA_ComInt_OutputH2N(pComOutputH, &stComOutputN);

    // 序列化
    uint8_t *pByteBuffer = (uint8_t *)&stComOutputN;
    int nRegisterNum = sizeof(SKA_ComAntiswayControllerOutput) / 2;
    for (int i = 0; i < nRegisterNum; i++)
    {
        pRegisterBuffer[i] = (uint16_t)((pByteBuffer[2 * i] << 8) | pByteBuffer[2 * i + 1]);
    }

    return 0;
}

int8_t SKA_ComInt_CreateParamJson(const SKA_ComAntiswayControllerParam *pComParam, cJSON **ppJson)
{
    /********************** 函数参数合法性检验 ********************/
    if (pComParam == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_CreateParamJson().");
        *ppJson = NULL;
        return -1;
    }

    /********************** 创建JSON对象 ********************/
    *ppJson = cJSON_CreateObject();
    if (*ppJson == NULL)
    {
        zlog_error(pCategory, "Failed to create JSON object.");
        return -1;
    }

    /********************** 通用参数 ********************/
    if (cJSON_AddNumberToObject(*ppJson, "SamplePeriod", pComParam->stGeneralParam.nSamplePeriod) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "MinEffRopeLength", pComParam->stGeneralParam.nMinEffRopeLength) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "MaxEffRopeLength", pComParam->stGeneralParam.nMaxEffRopeLength) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TerrainInflation", pComParam->stGeneralParam.nTerrainInflation) == NULL)
    {
        zlog_error(pCategory, "Failed to add General parameters to JSON object.");
        cJSON_Delete(*ppJson);
        *ppJson = NULL;
        return -1;
    }

    /********************** 大车参数 ********************/
    if (cJSON_AddNumberToObject(*ppJson, "BridgeMinPos", pComParam->stBridgeParam.nMinPos) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxPos", pComParam->stBridgeParam.nMaxPos) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxPosErr", pComParam->stBridgeParam.nMaxPosErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeStaticSpeed", pComParam->stBridgeParam.nStaticSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeAsLevel", pComParam->stBridgeParam.nAsLevel) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxOutFreq", pComParam->stBridgeParam.nMaxOutFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMinMoveFreq", pComParam->stBridgeParam.nMinMoveFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxOutSpeed", pComParam->stBridgeParam.nMaxOutSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxRefSpeed", pComParam->stBridgeParam.nMaxRefSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxOutAcc", pComParam->stBridgeParam.nMaxOutAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxRefAcc", pComParam->stBridgeParam.nMaxRefAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMinAssistSpeed", pComParam->stBridgeParam.nMinAssistSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgePrelimitSpeed", pComParam->stBridgeParam.nPrelimitSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeDelayTime", pComParam->stBridgeParam.nDelayTime) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeDriveOnDelay", pComParam->stBridgeParam.nDriveOnDelay) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxTrackErr", pComParam->stBridgeParam.nMaxTrackErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeTrackErrLim", pComParam->stBridgeParam.nTrackErrLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeTrackErrRateLim", pComParam->stBridgeParam.nTrackErrRateLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeTrackErrFilTc", pComParam->stBridgeParam.nTrackErrFilTc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgePosTrackKp", pComParam->stBridgeParam.nPosTrackKp) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgePosTrackKi", pComParam->stBridgeParam.nPosTrackKi) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeMaxResSway", pComParam->stBridgeParam.nMaxResSway) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "BridgeAntiswayKa", pComParam->stBridgeParam.nAntiswayKa) == NULL)
    {
        zlog_error(pCategory, "Failed to add Bridge parameters to JSON object.");
        cJSON_Delete(*ppJson);
        *ppJson = NULL;
        return -1;
    }

    /********************** 小车参数 ********************/
    if (cJSON_AddNumberToObject(*ppJson, "TrolleyMinPos", pComParam->stTrolleyParam.nMinPos) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxPos", pComParam->stTrolleyParam.nMaxPos) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxPosErr", pComParam->stTrolleyParam.nMaxPosErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyStaticSpeed", pComParam->stTrolleyParam.nStaticSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyAsLevel", pComParam->stTrolleyParam.nAsLevel) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxOutFreq", pComParam->stTrolleyParam.nMaxOutFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMinMoveFreq", pComParam->stTrolleyParam.nMinMoveFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxOutSpeed", pComParam->stTrolleyParam.nMaxOutSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxRefSpeed", pComParam->stTrolleyParam.nMaxRefSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxOutAcc", pComParam->stTrolleyParam.nMaxOutAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxRefAcc", pComParam->stTrolleyParam.nMaxRefAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMinAssistSpeed", pComParam->stTrolleyParam.nMinAssistSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyPrelimitSpeed", pComParam->stTrolleyParam.nPrelimitSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyDelayTime", pComParam->stTrolleyParam.nDelayTime) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyDriveOnDelay", pComParam->stTrolleyParam.nDriveOnDelay) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxTrackErr", pComParam->stTrolleyParam.nMaxTrackErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyTrackErrLim", pComParam->stTrolleyParam.nTrackErrLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyTrackErrRateLim", pComParam->stTrolleyParam.nTrackErrRateLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyTrackErrFilTc", pComParam->stTrolleyParam.nTrackErrFilTc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyPosTrackKp", pComParam->stTrolleyParam.nPosTrackKp) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyPosTrackKi", pComParam->stTrolleyParam.nPosTrackKi) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyMaxResSway", pComParam->stTrolleyParam.nMaxResSway) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "TrolleyAntiswayKa", pComParam->stTrolleyParam.nAntiswayKa) == NULL)
    {
        zlog_error(pCategory, "Failed to add Trolley parameters to JSON object.");
        cJSON_Delete(*ppJson);
        *ppJson = NULL;
        return -1;
    }

    /********************** 起升参数 ********************/
    if (cJSON_AddNumberToObject(*ppJson, "HoistMinRopeLen", pComParam->stHoistParam.nMinRopeLen) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxRopeLen", pComParam->stHoistParam.nMaxRopeLen) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxPosErr", pComParam->stHoistParam.nMaxPosErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistStaticSpeed", pComParam->stHoistParam.nStaticSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxOutFreq", pComParam->stHoistParam.nMaxOutFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMinMoveFreq", pComParam->stHoistParam.nMinMoveFreq) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxOutSpeed", pComParam->stHoistParam.nMaxOutSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxRefSpeed", pComParam->stHoistParam.nMaxRefSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxOutAcc", pComParam->stHoistParam.nMaxOutAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxRefAcc", pComParam->stHoistParam.nMaxRefAcc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistPrelimitSpeed", pComParam->stHoistParam.nPrelimitSpeed) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistDelayTime", pComParam->stHoistParam.nDelayTime) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistMaxTrackErr", pComParam->stHoistParam.nMaxTrackErr) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistTrackErrLim", pComParam->stHoistParam.nTrackErrLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistTrackErrRateLim", pComParam->stHoistParam.nTrackErrRateLim) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistTrackErrFilTc", pComParam->stHoistParam.nTrackErrFilTc) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistPosTrackKp", pComParam->stHoistParam.nPosTrackKp) == NULL ||
        cJSON_AddNumberToObject(*ppJson, "HoistPosTrackKi", pComParam->stHoistParam.nPosTrackKi) == NULL)
    {
        zlog_error(pCategory, "Failed to add Hoist parameters to JSON object.");
        cJSON_Delete(*ppJson);
        *ppJson = NULL;
        return -1;
    }

    return 0;
}

int8_t SKA_ComInt_ParseParamFromJson(const cJSON *pJson, SKA_ComAntiswayControllerParam *pComParam)
{
    /********************** 函数参数合法性检验 ********************/
    if (pJson == NULL || pComParam == NULL)
    {
        zlog_error(pCategory, "Invalid parameters of SKA_ComInt_ParseParamFromJson().");
        return -1;
    }

    cJSON *pJsonTmp = NULL;

    /********************** 通用参数 ********************/
    pJsonTmp = cJSON_GetObjectItem(pJson, "SamplePeriod");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get SamplePeriod from JSON object.");
        return -1;
    }
    pComParam->stGeneralParam.nSamplePeriod = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "MinEffRopeLength");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get MinEffRopeLength from JSON object.");
        return -1;
    }
    pComParam->stGeneralParam.nMinEffRopeLength = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "MaxEffRopeLength");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get MaxEffRopeLength from JSON object.");
        return -1;
    }
    pComParam->stGeneralParam.nMaxEffRopeLength = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TerrainInflation");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TerrainInflation from JSON object.");
        return -1;
    }
    pComParam->stGeneralParam.nTerrainInflation = pJsonTmp->valueint;

    /********************** 大车参数 ********************/
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMinPos");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMinPos from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMinPos = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxPos");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxPos from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxPos = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxPosErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxPosErr from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxPosErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeStaticSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeStaticSpeed from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nStaticSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeAsLevel");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeAsLevel from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nAsLevel = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxOutFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxOutFreq from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxOutFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMinMoveFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMinMoveFreq from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMinMoveFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxOutSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxOutSpeed from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxOutSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxRefSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxRefSpeed from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxRefSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxOutAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxOutAcc from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxOutAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxRefAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxRefAcc from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxRefAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMinAssistSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMinAssistSpeed from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMinAssistSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgePrelimitSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgePrelimitSpeed from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nPrelimitSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeDelayTime");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeDelayTime from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nDelayTime = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeDriveOnDelay");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeDriveOnDelay from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nDriveOnDelay = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxTrackErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxTrackErr from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxTrackErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeTrackErrLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeTrackErrLim from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nTrackErrLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeTrackErrRateLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeTrackErrRateLim from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nTrackErrRateLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeTrackErrFilTc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeTrackErrFilTc from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nTrackErrFilTc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgePosTrackKp");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgePosTrackKp from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nPosTrackKp = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgePosTrackKi");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgePosTrackKi from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nPosTrackKi = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeMaxResSway");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeMaxResSway from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nMaxResSway = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "BridgeAntiswayKa");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get BridgeAntiswayKa from JSON object.");
        return -1;
    }
    pComParam->stBridgeParam.nAntiswayKa = pJsonTmp->valueint;

    /********************** 小车参数 ********************/
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMinPos");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMinPos from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMinPos = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxPos");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxPos from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxPos = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxPosErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxPosErr from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxPosErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyStaticSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyStaticSpeed from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nStaticSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyAsLevel");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyAsLevel from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nAsLevel = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxOutFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxOutFreq from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxOutFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMinMoveFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMinMoveFreq from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMinMoveFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxOutSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxOutSpeed from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxOutSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxRefSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxRefSpeed from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxRefSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxOutAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxOutAcc from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxOutAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxRefAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxRefAcc from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxRefAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMinAssistSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMinAssistSpeed from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMinAssistSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyPrelimitSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyPrelimitSpeed from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nPrelimitSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyDelayTime");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyDelayTime from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nDelayTime = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyDriveOnDelay");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyDriveOnDelay from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nDriveOnDelay = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxTrackErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxTrackErr from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxTrackErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyTrackErrLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyTrackErrLim from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nTrackErrLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyTrackErrRateLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyTrackErrRateLim from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nTrackErrRateLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyTrackErrFilTc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyTrackErrFilTc from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nTrackErrFilTc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyPosTrackKp");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyPosTrackKp from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nPosTrackKp = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyPosTrackKi");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyPosTrackKi from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nPosTrackKi = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyMaxResSway");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyMaxResSway from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nMaxResSway = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "TrolleyAntiswayKa");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get TrolleyAntiswayKa from JSON object.");
        return -1;
    }
    pComParam->stTrolleyParam.nAntiswayKa = pJsonTmp->valueint;

    /********************** 起升参数 ********************/
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMinRopeLen");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMinRopeLen from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMinRopeLen = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxRopeLen");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxRopeLen from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxRopeLen = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxPosErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxPosErr from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxPosErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistStaticSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistStaticSpeed from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nStaticSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxOutFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxOutFreq from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxOutFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMinMoveFreq");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMinMoveFreq from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMinMoveFreq = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxOutSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxOutSpeed from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxOutSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxRefSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxRefSpeed from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxRefSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxOutAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxOutAcc from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxOutAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxRefAcc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxRefAcc from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxRefAcc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistPrelimitSpeed");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistPrelimitSpeed from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nPrelimitSpeed = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistDelayTime");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistDelayTime from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nDelayTime = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistMaxTrackErr");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistMaxTrackErr from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nMaxTrackErr = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistTrackErrLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistTrackErrLim from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nTrackErrLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistTrackErrRateLim");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistTrackErrRateLim from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nTrackErrRateLim = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistTrackErrFilTc");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistTrackErrFilTc from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nTrackErrFilTc = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistPosTrackKp");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistPosTrackKp from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nPosTrackKp = pJsonTmp->valueint;
    pJsonTmp = cJSON_GetObjectItem(pJson, "HoistPosTrackKi");
    if (pJsonTmp == NULL)
    {
        zlog_error(pCategory, "Failed to get HoistPosTrackKi from JSON object.");
        return -1;
    }
    pComParam->stHoistParam.nPosTrackKi = pJsonTmp->valueint;

    return 0;
}
