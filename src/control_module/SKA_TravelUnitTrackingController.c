/**
  ******************************************************************************
  * @file       : SKA_TravelUnitTrackingController.c
  * @brief      : 运行机构跟踪控制器
  * @author     : ZhangYi
  * @version    : None
  * @date       : 2024/10/24
  ******************************************************************************
  */
//

#include <stddef.h> 
#include <math.h>
#include <stdio.h> 

#include "SKA_Constants.h"
#include "SKA_TravelUnitTrackingController.h"

int8_t SKA_TUTC_Init(SKA_TravelUnitTrackingController *pTUTC, bool bActivePosCtrl, double dTs, double dDelayTime, double dStartPos,
                     double dMaxTrackErr, double dMaxTrackErrRate, double dTrackErrFilTc,
                     double dKp, double dKi, double dMaxVi, double dPosDeadZone,
                     double dMaxVelocity, double dMaxAcceleration, double dMaxFreq, double dMinFreq, double dFreqCompenSoft) {
    /******************** 函数参数合法性检验 ********************/
    if (pTUTC == NULL || dTs <= 0 || dDelayTime < 0 || dMaxTrackErr < 0 || dMaxTrackErrRate < 0 ||
        dTrackErrFilTc < 0 || dKp < 0 || dKi < 0 || dMaxVi < 0 || dPosDeadZone < 0 || 
        dMaxVelocity <= 0 || dMaxAcceleration <= 0 || dMaxFreq <= 0 || dMinFreq < 0 || dFreqCompenSoft < 0 || dFreqCompenSoft > 1) {
        printf("Invalid parameters of SKA_TUTC_Init().\n");
        return -1;
    }

    /******************** 初始化 ********************/

    pTUTC->bActivePosCtrl = bActivePosCtrl;
    pTUTC->dTs = dTs;

    SKA_IneFil_Init(&(pTUTC->stDelayer), dTs, dDelayTime, dStartPos);
    pTUTC->dMaxTrackErr = dMaxTrackErr;
    SKA_RatLim_Init(&(pTUTC->stTrackErrRL), dMaxTrackErrRate, -dMaxTrackErrRate, dTs, 0.0);
    SKA_IneFil_Init(&(pTUTC->stTrackErrIF), dTs, dTrackErrFilTc, 0.0);

    pTUTC->dKp = dKp;
    pTUTC->dKi = dKi;
    pTUTC->dMaxVi = dMaxVi;
    pTUTC->dPosDeadZone = dPosDeadZone;
    pTUTC->bOpenIntegration = 0;
    pTUTC->dTrackErrInteg = 0.0;

    pTUTC->dMaxVelocity = dMaxVelocity;
    SKA_RatLim_Init(&(pTUTC->stVelocityRL), dMaxAcceleration, -dMaxAcceleration, dTs, 0.0);

    pTUTC->dMaxFreq = dMaxFreq;
    pTUTC->dMinFreq = dMinFreq;
    pTUTC->dFreqCompenSoft = dFreqCompenSoft;

    return 0;
}

int8_t SKA_TUTC_Run(SKA_TravelUnitTrackingController *pTUTC,
              double dRefPos, double dRefVel, double dRefAcc, double dCurPos,
              double *pOutputVel, double *pOutputFreq, int8_t *pDirection) {
    /******************** 函数参数合法性检验 ********************/
    if (pTUTC == NULL || pOutputVel == NULL || pOutputFreq == NULL || pDirection == NULL) {
        printf("Invalid parameters of SKA_TUTC_Run().\n");
        return -1;
    }

    /******************** 运行跟踪控制器 ******************** */

    /** 前馈控制 **/

    // 前馈控制量
    double dFeedforward = dRefVel;

    /** 反馈控制 **/

    // 延迟时间补偿
    double dRefPosDc;
    SKA_IneFil_Run(&(pTUTC->stDelayer), dRefPos, &dRefPosDc);
    
    // 位置误差预处理
    // 原始位置误差
    pTUTC->dOriTrackError = dRefPosDc - dCurPos;
    // 限幅滤波
    double dSatTrackError;
    SKA_Sat_Run(pTUTC->dOriTrackError, -pTUTC->dMaxTrackErr, pTUTC->dMaxTrackErr, &dSatTrackError);
    // 限速滤波
    double fRatLimTrackErr;
    SKA_RatLim_Run(&(pTUTC->stTrackErrRL), dSatTrackError, &fRatLimTrackErr);
    // 惯性滤波
    SKA_IneFil_Run(&(pTUTC->stTrackErrIF), fRatLimTrackErr, &(pTUTC->dFilTrackError));

    double dFeedback;
    if (pTUTC->bActivePosCtrl){
        // 激活闭环定位
        
        // PI控制器
        // 比例环节
        pTUTC->dVp = pTUTC->dKp * pTUTC->dFilTrackError;
        // 积分环节
        // 积分分离
        if (pTUTC->bOpenIntegration == 1) {
            // 启动积分
            // 避免积分饱和
            if (fabs(pTUTC->dKi * pTUTC->dTrackErrInteg) < pTUTC->dMaxVi) {
                pTUTC->dTrackErrInteg += pTUTC->dTs * pTUTC->dFilTrackError;
            } else {
                if (pTUTC->dFilTrackError * pTUTC->dTrackErrInteg < 0) {
                    pTUTC->dTrackErrInteg += pTUTC->dTs * pTUTC->dFilTrackError;
                } else {
                    // 积分饱和
                }
            }
        } else {
            // 关闭积分
            pTUTC->dTrackErrInteg = 0.0;
        }
        pTUTC->dVi = pTUTC->dKi * pTUTC->dTrackErrInteg;

        // 反馈控制量
        if(fabs(pTUTC->dFilTrackError) < pTUTC->dPosDeadZone){
            // 反馈控制死区
            dFeedback = 0.0;
            pTUTC->dTrackErrInteg = 0.0;
        }else{
            dFeedback = pTUTC->dVp + pTUTC->dVi;
        }
    } else {
        // 未激活闭环定位
        dFeedback = 0.0;
    }

    /** 控制量处理 **/

    // 复合控制量
    double dVsum = dFeedforward + dFeedback;
    // 速度限制
    double dVsat;
    SKA_Sat_Run(dVsum, -pTUTC->dMaxVelocity, pTUTC->dMaxVelocity, &dVsat);
    // 加速度限制
    double dVrl;
    SKA_RatLim_Run(&(pTUTC->stVelocityRL), dVsat, &dVrl);
    // 速度控制量
    double dUv = fabs(dVrl) < SKA_FlOAT_ERROR ? 0.0 : dVrl;
    
    // 输出运动方向
    if (dUv == 0.0) {
        *pDirection = 0;
    } else if (dUv > 0) {
        *pDirection = 1;
    } else {
        *pDirection = -1;
    }
    // 输出速度和频率
    // 速度-频率比例系数
    double fRatio = pTUTC->dMaxVelocity / pTUTC->dMaxFreq;
    *pOutputVel = dUv;
    // 控制量死区补偿
    if (*pDirection != 0) {
        // 最小能动频率对应的速度
        double dMinVelocity = pTUTC->dMinFreq * fRatio;
        if(fabs(dUv) < dMinVelocity){
            *pOutputVel = *pDirection * dMinVelocity * pow(fabs(dUv) / dMinVelocity, pTUTC->dFreqCompenSoft);
        }
    }
    *pOutputFreq = *pOutputVel / fRatio;

    return 0;
}

int8_t SKA_TUTC_GetPosTrackError(SKA_TravelUnitTrackingController *pTUTC, double *pPosTrackError) {
    /******************** 函数参数合法性检验 ********************/
    if (pTUTC == NULL || pPosTrackError == NULL) {
        printf("Invalid parameters of SKA_TUTC_GetPosTrackError().\n");
        return -1;
    }

    /******************** 获取位置跟踪误差 ******************** */
    if (pTUTC->bActivePosCtrl) {
        // 激活闭环定位控制
        *pPosTrackError = pTUTC->dOriTrackError;
    } else {
        // 未激活闭环定位控制
        *pPosTrackError = 0.0;
    }

    return 0;
}