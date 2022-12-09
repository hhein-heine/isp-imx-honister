/******************************************************************************\
|* Copyright (c) 2020 by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")       *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/
/**
 * @file AR1335_priv.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroupar1335_priv
 * @{
 *
 */
#ifndef __AR1335_PRIV_H__
#define __AR1335_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>
#include <isi/isi_common.h>
#include "vvsensor.h"



#ifdef __cplusplus
extern "C"
{
#endif



/*****************************************************************************
 * SC control registers
 *****************************************************************************/
#define AR1335_PIDH                         (0x300A)  //R  - Product ID High Byte MSBs
#define AR1335_PIDL                         (0x300B)  //R  - Product ID Low Byte LSBs

/*****************************************************************************
 * Default values
 *****************************************************************************/

 // Make sure that these static settings are reflecting the capabilities defined
// in IsiGetCapsIss (further dynamic setup may alter these default settings but
// often does not if there is no choice available).

/*****************************************************************************
 * SC control registers
 *****************************************************************************/
#define AR1335_PIDH_DEFAULT                        (0x27) //read only
#define AR1335_PIDL_DEFAULT                        (0x70) //read only

typedef struct AR1335_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    struct vvcam_mode_info SensorMode;
    uint32_t            KernelDriverFlag;
    char                SensorRegCfgFile[64];

    uint32_t              HdrMode;
    uint32_t              Resolution;
    uint32_t              MaxFps;
    uint32_t              MinFps;
    uint32_t              CurrFps;
    //// modify below here ////

    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that csensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    bool_t              isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    float               one_line_exp_time;
    uint16_t            MaxIntegrationLine;
    uint16_t            MinIntegrationLine;
    uint32_t            gain_accuracy;

    uint16_t            FrameLengthLines;       /**< frame line length */
    uint16_t            CurFrameLengthLines;

    float               AecMinGain;
    float               AecMaxGain;
    float               AecMinIntegrationTime;
    float               AecMaxIntegrationTime;

    float               AecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    float               AecCurIntegrationTime;
    float               AecCurVSIntegrationTime;
    float               AecCurLongIntegrationTime;
    float               AecCurGain;
    float               AecCurVSGain;
    float               AecCurLongGain;
    // added 02/02
    uint32_t            LastExpLine;
    uint32_t            LastVsExpLine;
    uint32_t            LastLongExpLine;

    uint32_t            LastGain;
    uint32_t            LastVsGain;
    uint32_t            LastLongGain;

    bool                GroupHold;
    uint32_t            OldGain;
    uint32_t            OldVsGain;
    uint32_t            OldIntegrationTime;
    uint32_t            OldVsIntegrationTime;
    uint32_t            OldGainHcg;
    uint32_t            OldAGainHcg;
    uint32_t            OldGainLcg;
    uint32_t            OldAGainLcg;
    int                 subdev;
    // bool                enableHdr;
    uint8_t             pattern;

    float               CurHdrRatio;
} AR1335_Context_t;

static RESULT AR1335_IsiCreateSensorIss(IsiSensorInstanceConfig_t *
                          pConfig);

static RESULT AR1335_IsiInitSensorIss(IsiSensorHandle_t handle);

static RESULT AR1335_IsiReleaseSensorIss(IsiSensorHandle_t handle);

static RESULT AR1335_IsiGetCapsIss(IsiSensorHandle_t handle,
                         IsiSensorCaps_t * pIsiSensorCaps);

static RESULT AR1335_IsiSetupSensorIss(IsiSensorHandle_t handle,
                         const IsiSensorConfig_t *
                         pConfig);

static RESULT AR1335_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                               bool_t on);

static RESULT AR1335_IsiSensorSetPowerIss(IsiSensorHandle_t handle,
                            bool_t on);

static RESULT AR1335_IsiGetSensorRevisionIss(IsiSensorHandle_t handle,
                               uint32_t * p_value);

static RESULT AR1335_IsiSetBayerPattern(IsiSensorHandle_t handle,
                          uint8_t pattern);

static RESULT AR1335_IsiGetGainLimitsIss(IsiSensorHandle_t handle,
                             float *pMinGain,
                             float *pMaxGain);

static RESULT AR1335_IsiGetIntegrationTimeLimitsIss(IsiSensorHandle_t
                                 handle,
                                 float
                                 *pMinIntegrationTime,
                                 float
                                 *pMaxIntegrationTime);

static RESULT AR1335_IsiExposureControlIss(IsiSensorHandle_t handle,
                            float NewGain,
                            float NewIntegrationTime,
                            uint8_t *
                            pNumberOfFramesToSkip,
                            float *pSetGain,
                            float *pSetIntegrationTime,
                            float *hdr_ratio);

static RESULT AR1335_IsiGetGainIss(IsiSensorHandle_t handle,
                        float *pSetGain);

static RESULT AR1335_IsiGetVSGainIss(IsiSensorHandle_t handle,
                          float *pSetGain);

static RESULT AR1335_IsiGetGainIncrementIss(IsiSensorHandle_t handle,
                             float *pIncr);

static RESULT AR1335_IsiSetGainIss(IsiSensorHandle_t handle,
                        float NewGain, float *pSetGain,
                        float *hdr_ratio);

#if 0
static RESULT AR1335_IsiSetVSGainIss(IsiSensorHandle_t handle,
                          float NewIntegrationTime,
                          float NewGain, float *pSetGain,
                          float *hdr_ratio);
#endif

static RESULT AR1335_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                               float
                               *pSetIntegrationTime);

static RESULT AR1335_IsiGetVSIntegrationTimeIss(IsiSensorHandle_t
                             handle,
                             float
                             *pSetIntegrationTime);

static RESULT AR1335_IsiGetIntegrationTimeIncrementIss(IsiSensorHandle_t handle,
                             float *pIncr);

static RESULT AR1335_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                               float NewIntegrationTime,
                               float
                               *pSetIntegrationTime,
                               uint8_t *
                               pNumberOfFramesToSkip,
                               float *hdr_ratio);

#if 0
static RESULT AR1335_IsiSetVSIntegrationTimeIss(IsiSensorHandle_t
                             handle,
                             float
                             NewIntegrationTime,
                             float
                             *pSetIntegrationTime,
                             uint8_t *
                             pNumberOfFramesToSkip,
                             float *hdr_ratio);
#endif

RESULT AR1335_IsiGetResolutionIss(IsiSensorHandle_t handle, uint16_t *pwidth, uint16_t *pheight);

static RESULT AR1335_IsiResetSensorIss(IsiSensorHandle_t handle);


#ifdef __cplusplus
}
#endif

/* @}ar1335priv */

#endif    /* __AR1335PRIV_H__ */

