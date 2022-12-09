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

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/return_codes.h>
#include <common/misc.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "vvsensor.h"

CREATE_TRACER( OV2775_INFO , "OV2775: ", INFO,    0);
CREATE_TRACER( OV2775_WARN , "OV2775: ", WARNING, 0);
CREATE_TRACER( OV2775_ERROR, "OV2775: ", ERROR,   1);

#ifdef SUBDEV_V4L2
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-subdev.h>
//#undef TRACE
//#define TRACE(x, ...)
#endif

static const char SensorName[16] = "ov2775";

typedef struct OV2755_Context_s
{
    IsiSensorContext_t  IsiCtx;
    struct vvcam_mode_info_s CurMode;
    IsiSensorAeInfo_t AeInfo;
    IsiSensorIntTime_t IntTime;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    IsiSensorGain_t SensorGain;
    uint32_t minAfps;
    uint64_t AEStartExposure;
} OV2775_Context_t;

static RESULT OV2775_IsiSensorSetPowerIss(IsiSensorHandle_t handle, bool_t on)
{
    int ret = 0;

    TRACE( OV2775_INFO, "%s: (enter)\n", __func__);
    TRACE( OV2775_INFO, "%s: set power %d\n", __func__,on);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    int32_t power = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_POWER, &power);
    if (ret != 0){
        TRACE(OV2775_ERROR, "%s set power %d error\n", __func__,power);
        return RET_FAILURE;
    }

    TRACE( OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorGetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (!pclk)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CLK, pclk);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s get clock error\n", __func__);
        return RET_FAILURE;
    } 
    
    TRACE( OV2775_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);
    TRACE( OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorSetClkIss(IsiSensorHandle_t handle,
                                        struct vvcam_clk_s *pclk)
{
    int ret = 0;

    TRACE( OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pclk == NULL)
        return RET_NULL_POINTER;
    
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_CLK, &pclk);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( OV2775_INFO, "%s: status:%d sensor_mclk:%d csi_max_pixel_clk:%d\n",
        __func__, pclk->status, pclk->sensor_mclk, pclk->csi_max_pixel_clk);

    TRACE( OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiResetSensorIss(IsiSensorHandle_t handle)
{
    int ret = 0;

    TRACE( OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_RESET, NULL);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s set reset error\n", __func__);
        return RET_FAILURE;
    }

    TRACE( OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiRegisterReadIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        uint32_t * pValue)
{
    int32_t ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = 0;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_READ_REG, &sccb_data);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: read sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    *pValue = sccb_data.data;

    TRACE(OV2775_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiRegisterWriteIss(IsiSensorHandle_t handle,
                                        const uint32_t address,
                                        const uint32_t value)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    struct vvcam_sccb_data_s sccb_data;
    sccb_data.addr = address;
    sccb_data.data = value;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_WRITE_REG, &sccb_data);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: write sensor register error!\n", __func__);
        return (RET_FAILURE);
    }

    TRACE(OV2775_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_UpdateIsiAEInfo(IsiSensorHandle_t handle)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    uint32_t exp_line_time = pOV2775Ctx->CurMode.ae_info.one_line_exp_time_ns;

    IsiSensorAeInfo_t *pAeInfo = &pOV2775Ctx->AeInfo;
    pAeInfo->oneLineExpTime = (exp_line_time << ISI_EXPO_PARAS_FIX_FRACBITS) / 1000;

    if (pOV2775Ctx->CurMode.hdr_mode == SENSOR_MODE_LINEAR) {
        pAeInfo->maxIntTime.linearInt =
            pOV2775Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->minIntTime.linearInt =
            pOV2775Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;
        pAeInfo->maxAGain.linearGainParas = pOV2775Ctx->CurMode.ae_info.max_again;
        pAeInfo->minAGain.linearGainParas = pOV2775Ctx->CurMode.ae_info.min_again;
        pAeInfo->maxDGain.linearGainParas = pOV2775Ctx->CurMode.ae_info.max_dgain;
        pAeInfo->minDGain.linearGainParas = pOV2775Ctx->CurMode.ae_info.min_dgain;
    } else {
        switch (pOV2775Ctx->CurMode.stitching_mode) {
            case SENSOR_STITCHING_DUAL_DCG:
            case SENSOR_STITCHING_3DOL:
            case SENSOR_STITCHING_LINEBYLINE:
                pAeInfo->maxIntTime.triInt.triSIntTime =
                    pOV2775Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triSIntTime =
                    pOV2775Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                
                pAeInfo->maxIntTime.triInt.triIntTime =
                    pOV2775Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.triInt.triIntTime =
                    pOV2775Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pOV2775Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG) {
                    pAeInfo->maxIntTime.triInt.triLIntTime = pAeInfo->maxIntTime.triInt.triIntTime;
                    pAeInfo->minIntTime.triInt.triLIntTime = pAeInfo->minIntTime.triInt.triIntTime;
                } else {
                    pAeInfo->maxIntTime.triInt.triLIntTime =
                        pOV2775Ctx->CurMode.ae_info.max_longintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.triInt.triLIntTime =
                        pOV2775Ctx->CurMode.ae_info.min_longintegration_line * pAeInfo->oneLineExpTime;
                }

                pAeInfo->maxAGain.triGainParas.triSGain = pOV2775Ctx->CurMode.ae_info.max_short_again;
                pAeInfo->minAGain.triGainParas.triSGain = pOV2775Ctx->CurMode.ae_info.min_short_again;
                pAeInfo->maxDGain.triGainParas.triSGain = pOV2775Ctx->CurMode.ae_info.max_short_dgain;
                pAeInfo->minDGain.triGainParas.triSGain = pOV2775Ctx->CurMode.ae_info.min_short_dgain;

                pAeInfo->maxAGain.triGainParas.triGain = pOV2775Ctx->CurMode.ae_info.max_again;
                pAeInfo->minAGain.triGainParas.triGain = pOV2775Ctx->CurMode.ae_info.min_again;
                pAeInfo->maxDGain.triGainParas.triGain = pOV2775Ctx->CurMode.ae_info.max_dgain;
                pAeInfo->minDGain.triGainParas.triGain = pOV2775Ctx->CurMode.ae_info.min_dgain;

                pAeInfo->maxAGain.triGainParas.triLGain = pOV2775Ctx->CurMode.ae_info.max_long_again;
                pAeInfo->minAGain.triGainParas.triLGain = pOV2775Ctx->CurMode.ae_info.min_long_again;
                pAeInfo->maxDGain.triGainParas.triLGain = pOV2775Ctx->CurMode.ae_info.max_long_dgain;
                pAeInfo->minDGain.triGainParas.triLGain = pOV2775Ctx->CurMode.ae_info.min_long_dgain;
                break;
            case SENSOR_STITCHING_DUAL_DCG_NOWAIT:
            case SENSOR_STITCHING_16BIT_COMPRESS:
            case SENSOR_STITCHING_L_AND_S:
            case SENSOR_STITCHING_2DOL:
                pAeInfo->maxIntTime.dualInt.dualIntTime =
                    pOV2775Ctx->CurMode.ae_info.max_integration_line * pAeInfo->oneLineExpTime;
                pAeInfo->minIntTime.dualInt.dualIntTime =
                    pOV2775Ctx->CurMode.ae_info.min_integration_line * pAeInfo->oneLineExpTime;

                if (pOV2775Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime = pAeInfo->maxIntTime.dualInt.dualIntTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime = pAeInfo->minIntTime.dualInt.dualIntTime;
                } else {
                    pAeInfo->maxIntTime.dualInt.dualSIntTime =
                        pOV2775Ctx->CurMode.ae_info.max_vsintegration_line * pAeInfo->oneLineExpTime;
                    pAeInfo->minIntTime.dualInt.dualSIntTime =
                        pOV2775Ctx->CurMode.ae_info.min_vsintegration_line * pAeInfo->oneLineExpTime;
                }
                
                if (pOV2775Ctx->CurMode.stitching_mode == SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.min_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.max_long_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.min_long_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.max_long_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.min_long_dgain;
                } else {
                    pAeInfo->maxAGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.max_short_again;
                    pAeInfo->minAGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.min_short_again;
                    pAeInfo->maxDGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.max_short_dgain;
                    pAeInfo->minDGain.dualGainParas.dualSGain = pOV2775Ctx->CurMode.ae_info.min_short_dgain;
                    pAeInfo->maxAGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.max_again;
                    pAeInfo->minAGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.min_again;
                    pAeInfo->maxDGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.max_dgain;
                    pAeInfo->minDGain.dualGainParas.dualGain  = pOV2775Ctx->CurMode.ae_info.min_dgain;
                }
                
                break;
            default:
                break;
        }
    }
    pAeInfo->gainStep = pOV2775Ctx->CurMode.ae_info.gain_step;
    pAeInfo->currFps  = pOV2775Ctx->CurMode.ae_info.cur_fps;
    pAeInfo->maxFps   = pOV2775Ctx->CurMode.ae_info.max_fps;
    pAeInfo->minFps   = pOV2775Ctx->CurMode.ae_info.min_fps;
    pAeInfo->minAfps  = pOV2775Ctx->CurMode.ae_info.min_afps;
    pAeInfo->hdrRatio[0] = pOV2775Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s;
    pAeInfo->hdrRatio[1] = pOV2775Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs;

    pAeInfo->intUpdateDlyFrm = pOV2775Ctx->CurMode.ae_info.int_update_delay_frm;
    pAeInfo->gainUpdateDlyFrm = pOV2775Ctx->CurMode.ae_info.gain_update_delay_frm;

    if (pOV2775Ctx->minAfps != 0) {
        pAeInfo->minAfps = pOV2775Ctx->minAfps;
    } 
    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    memcpy(pMode, &pOV2775Ctx->CurMode, sizeof(IsiSensorMode_t));

    TRACE(OV2775_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetSensorModeIss(IsiSensorHandle_t handle,
                                         IsiSensorMode_t *pMode)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pMode == NULL)
        return (RET_NULL_POINTER);

    struct vvcam_mode_info_s sensor_mode;
    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    sensor_mode.index = pMode->index;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    memset(&sensor_mode, 0, sizeof(struct vvcam_mode_info_s));
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s set sensor mode failed", __func__);
        return RET_FAILURE;
    }
    memcpy(&pOV2775Ctx->CurMode, &sensor_mode, sizeof(struct vvcam_mode_info_s));
    OV2775_UpdateIsiAEInfo(handle);

    TRACE(OV2775_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorSetStreamingIss(IsiSensorHandle_t handle,
                                              bool_t on)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    uint32_t status = on;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_STREAM, &status);
    if (ret != 0){
        TRACE(OV2775_ERROR, "%s set sensor stream %d error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: set streaming %d\n", __func__, on);
    TRACE(OV2775_INFO, "%s (exit) \n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiCreateSensorIss(IsiSensorInstanceConfig_t * pConfig)
{
    RESULT result = RET_SUCCESS;
    OV2775_Context_t *pOV2775Ctx;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    if (!pConfig || !pConfig->pSensor || !pConfig->HalHandle)
        return RET_NULL_POINTER;

    pOV2775Ctx = (OV2775_Context_t *) malloc(sizeof(OV2775_Context_t));
    if (!pOV2775Ctx)
        return RET_OUTOFMEM;

    memset(pOV2775Ctx, 0, sizeof(OV2775_Context_t));
    pOV2775Ctx->IsiCtx.HalHandle = pConfig->HalHandle;
    pOV2775Ctx->IsiCtx.pSensor   = pConfig->pSensor;
    pConfig->hSensor = (IsiSensorHandle_t) pOV2775Ctx;

    result = OV2775_IsiSensorSetPowerIss(pOV2775Ctx, BOOL_TRUE);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s set power error\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    result = OV2775_IsiSensorGetClkIss(pOV2775Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s get clk error\n", __func__);
        return RET_FAILURE;
    }
    clk.status = 1;
    result = OV2775_IsiSensorSetClkIss(pOV2775Ctx, &clk);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s set clk error\n", __func__);
        return RET_FAILURE;
    }
    result = OV2775_IsiResetSensorIss(pOV2775Ctx);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s retset sensor error\n", __func__);
        return RET_FAILURE;
    }

    IsiSensorMode_t SensorMode;
    SensorMode.index = pConfig->SensorModeIndex;
    result = OV2775_IsiSetSensorModeIss(pOV2775Ctx, &SensorMode);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s set sensor mode error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV2775_IsiReleaseSensorIss(IsiSensorHandle_t handle)
{
    TRACE(OV2775_INFO, "%s (enter) \n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    if (pOV2775Ctx == NULL)
        return (RET_WRONG_HANDLE);

    OV2775_IsiSensorSetStreamingIss(pOV2775Ctx, BOOL_FALSE);
    struct vvcam_clk_s clk;
    memset(&clk, 0, sizeof(struct vvcam_clk_s));
    OV2775_IsiSensorGetClkIss(pOV2775Ctx, &clk);
    clk.status = 0;
    OV2775_IsiSensorSetClkIss(pOV2775Ctx, &clk);
    OV2775_IsiSensorSetPowerIss(pOV2775Ctx, BOOL_FALSE);
    free(pOV2775Ctx);
    pOV2775Ctx = NULL;

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiHalQuerySensorIss(HalHandle_t HalHandle,
                                          IsiSensorModeInfoArray_t *pSensorMode)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter) \n", __func__);

    if (HalHandle == NULL || pSensorMode == NULL)
        return RET_NULL_POINTER;

    HalContext_t *pHalCtx = (HalContext_t *)HalHandle;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_QUERY, pSensorMode);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiQuerySensorIss(IsiSensorHandle_t handle,
                                       IsiSensorModeInfoArray_t *pSensorMode)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV2775_INFO, "%s (enter) \n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    result = OV2775_IsiHalQuerySensorIss(pOV2775Ctx->IsiCtx.HalHandle,
                                         pSensorMode);
    if (result != RET_SUCCESS)
        TRACE(OV2775_ERROR, "%s: query sensor mode info error!\n", __func__);

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV2775_IsiGetCapsIss(IsiSensorHandle_t handle,
                                   IsiSensorCaps_t * pIsiSensorCaps)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV2775_INFO, "%s (enter) \n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    IsiSensorModeInfoArray_t SensorModeInfo;
    memset(&SensorModeInfo, 0, sizeof(IsiSensorModeInfoArray_t));
    result = OV2775_IsiQuerySensorIss(handle, &SensorModeInfo);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s: query sensor mode info error!\n", __func__);
        return RET_FAILURE;
    }

    pIsiSensorCaps->FieldSelection    = ISI_FIELDSEL_BOTH;
    pIsiSensorCaps->YCSequence        = ISI_YCSEQ_YCBYCR;
    pIsiSensorCaps->Conv422           = ISI_CONV422_NOCOSITED;
    pIsiSensorCaps->HPol              = ISI_HPOL_REFPOS;
    pIsiSensorCaps->VPol              = ISI_VPOL_NEG;
    pIsiSensorCaps->Edge              = ISI_EDGE_RISING;
    pIsiSensorCaps->supportModeNum    = SensorModeInfo.count;
    pIsiSensorCaps->currentMode       = pOV2775Ctx->CurMode.index;

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return result;
}

static RESULT OV2775_IsiSetupSensorIss(IsiSensorHandle_t handle,
                                       const IsiSensorCaps_t *pIsiSensorCaps )
{
    int ret = 0;
    RESULT result = RET_SUCCESS;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pIsiSensorCaps == NULL)
        return RET_NULL_POINTER;

    if (pIsiSensorCaps->currentMode != pOV2775Ctx->CurMode.index) {
        IsiSensorMode_t SensorMode;
        memset(&SensorMode, 0, sizeof(IsiSensorMode_t));
        SensorMode.index = pIsiSensorCaps->currentMode;
        result = OV2775_IsiSetSensorModeIss(handle, &SensorMode);
        if (result != RET_SUCCESS) {
            TRACE(OV2775_ERROR, "%s:set sensor mode %d failed!\n",
                  __func__, SensorMode.index);
            return result;
        }
    }

#ifdef SUBDEV_V4L2
    struct v4l2_subdev_format format;
    memset(&format, 0, sizeof(struct v4l2_subdev_format));
    format.format.width  = pOV2775Ctx->CurMode.size.bounds_width;
    format.format.height = pOV2775Ctx->CurMode.size.bounds_height;
    format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    format.pad = 0;
    ret = ioctl(pHalCtx->sensor_fd, VIDIOC_SUBDEV_S_FMT, &format);
    if (ret != 0){
        TRACE(OV2775_ERROR, "%s: sensor set format error!\n", __func__);
        return RET_FAILURE;
    }
#else
    ret = ioctrl(pHalCtx->sensor_fd, VVSENSORIOC_S_INIT, NULL);
    if (ret != 0){
        TRACE(OV2775_ERROR, "%s: sensor init error!\n", __func__);
        return RET_FAILURE;
    }
#endif

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetSensorRevisionIss(IsiSensorHandle_t handle, uint32_t *pValue)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pValue == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_CHIP_ID, pValue);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: get chip id error!\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiCheckSensorConnectionIss(IsiSensorHandle_t handle)
{
    RESULT result = RET_SUCCESS;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    uint32_t ChipId = 0;
    result = OV2775_IsiGetSensorRevisionIss(handle, &ChipId);
    if (result != RET_SUCCESS) {
        TRACE(OV2775_ERROR, "%s:get sensor chip id error!\n",__func__);
        return RET_FAILURE;
    }

    if (ChipId != 0x2770) {
        TRACE(OV2775_ERROR,
            "%s:ChipID=0x2770,while read sensor Id=0x%x error!\n",
             __func__, ChipId);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetAeInfoIss(IsiSensorHandle_t handle,
                                     IsiSensorAeInfo_t *pAeInfo)
{
    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    if (pAeInfo == NULL)
        return RET_NULL_POINTER;

    memcpy(pAeInfo, &pOV2775Ctx->AeInfo, sizeof(IsiSensorAeInfo_t));

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetHdrRatioIss(IsiSensorHandle_t handle,
                                       uint8_t hdrRatioNum,
                                       uint32_t HdrRatio[])
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    struct sensor_hdr_artio_s hdr_ratio;
    if (hdrRatioNum == 2) {
        hdr_ratio.ratio_s_vs = HdrRatio[1];
        hdr_ratio.ratio_l_s = HdrRatio[0];
    }else {
        hdr_ratio.ratio_s_vs = HdrRatio[0];
        hdr_ratio.ratio_l_s = 0;
    }

    if (hdr_ratio.ratio_s_vs == pOV2775Ctx->CurMode.ae_info.hdr_ratio.ratio_s_vs &&
        hdr_ratio.ratio_l_s == pOV2775Ctx->CurMode.ae_info.hdr_ratio.ratio_l_s)
        return RET_SUCCESS;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_HDR_RADIO, &hdr_ratio);
    if (ret != 0) {
        TRACE(OV2775_ERROR,"%s: set hdr ratio error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s sensor_mode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &sensor_mode);
    if (ret != 0) {
        TRACE(OV2775_ERROR,"%s: get mode info error!\n", __func__);
        return RET_FAILURE;
    }

    memcpy(&pOV2775Ctx->CurMode, &sensor_mode, sizeof (struct vvcam_mode_info_s));
    OV2775_UpdateIsiAEInfo(handle);

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    memcpy(pIntegrationTime, &pOV2775Ctx->IntTime, sizeof(IsiSensorIntTime_t));

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;

}

static RESULT OV2775_IsiSetIntegrationTimeIss(IsiSensorHandle_t handle,
                                   IsiSensorIntTime_t *pIntegrationTime)
{
    int ret = 0;
    uint32_t LongIntLine;
    uint32_t IntLine;
    uint32_t ShortIntLine;
    uint32_t oneLineTime;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pIntegrationTime == NULL)
        return RET_NULL_POINTER;

    oneLineTime =  pOV2775Ctx->AeInfo.oneLineExpTime;
    pOV2775Ctx->IntTime.expoFrmType = pIntegrationTime->expoFrmType;

    switch (pIntegrationTime->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            IntLine = (pIntegrationTime->IntegrationTime.linearInt +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pOV2775Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor linear exp error!\n", __func__);
                    return RET_FAILURE;
                }
               pOV2775Ctx->IntLine = IntLine;
            }
            TRACE(OV2775_INFO, "%s set linear exp %d \n", __func__,IntLine);
            pOV2775Ctx->IntTime.IntegrationTime.linearInt =  IntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            IntLine = (pIntegrationTime->IntegrationTime.dualInt.dualIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pOV2775Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor dual exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pOV2775Ctx->IntLine = IntLine;
            }

            if (pOV2775Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                ShortIntLine = (pIntegrationTime->IntegrationTime.dualInt.dualSIntTime +
                               (oneLineTime / 2)) / oneLineTime;
                if (ShortIntLine != pOV2775Ctx->ShortIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                    if (ret != 0) {
                        TRACE(OV2775_ERROR,"%s:set sensor dual vsexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pOV2775Ctx->ShortIntLine = ShortIntLine;
                }
            } else {
                ShortIntLine = IntLine;
                pOV2775Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(OV2775_INFO, "%s set dual exp %d short_exp %d\n", __func__, IntLine, ShortIntLine);
            pOV2775Ctx->IntTime.IntegrationTime.dualInt.dualIntTime  = IntLine * oneLineTime;
            pOV2775Ctx->IntTime.IntegrationTime.dualInt.dualSIntTime = ShortIntLine * oneLineTime;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if (pOV2775Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triLIntTime +
                        (oneLineTime / 2)) / oneLineTime;
                if (LongIntLine != pOV2775Ctx->LongIntLine) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_EXP, &LongIntLine);
                    if (ret != 0) {
                        TRACE(OV2775_ERROR,"%s:set sensor tri lexp error!\n", __func__);
                        return RET_FAILURE;
                    }
                    pOV2775Ctx->LongIntLine = LongIntLine;
                }
            } else {
                LongIntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
                pOV2775Ctx->LongIntLine = LongIntLine;
            }

            IntLine = (pIntegrationTime->IntegrationTime.triInt.triIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (IntLine != pOV2775Ctx->IntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_EXP, &IntLine);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor tri exp error!\n", __func__);
                    return RET_FAILURE;
                }
                pOV2775Ctx->IntLine = IntLine;
            }
            
            ShortIntLine = (pIntegrationTime->IntegrationTime.triInt.triSIntTime +
                       (oneLineTime / 2)) / oneLineTime;
            if (ShortIntLine != pOV2775Ctx->ShortIntLine) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSEXP, &ShortIntLine);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor tri vsexp error!\n", __func__);
                    return RET_FAILURE;
                }
                pOV2775Ctx->ShortIntLine = ShortIntLine;
            }
            TRACE(OV2775_INFO, "%s set tri long exp %d exp %d short_exp %d\n", __func__, LongIntLine, IntLine, ShortIntLine);
            pOV2775Ctx->IntTime.IntegrationTime.triInt.triLIntTime = LongIntLine * oneLineTime;
            pOV2775Ctx->IntTime.IntegrationTime.triInt.triIntTime = IntLine * oneLineTime;
            pOV2775Ctx->IntTime.IntegrationTime.triInt.triSIntTime = ShortIntLine * oneLineTime;
            break;
        default:
            return RET_FAILURE;
            break;
    }
    
    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    if (pGain == NULL)
        return RET_NULL_POINTER;
    memcpy(pGain, &pOV2775Ctx->SensorGain, sizeof(IsiSensorGain_t));

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetGainIss(IsiSensorHandle_t handle, IsiSensorGain_t *pGain)
{
    int ret = 0;
    uint32_t LongGain;
    uint32_t Gain;
    uint32_t ShortGain;

    TRACE(OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pGain == NULL)
        return RET_NULL_POINTER;

    pOV2775Ctx->SensorGain.expoFrmType = pGain->expoFrmType;
    switch (pGain->expoFrmType) {
        case ISI_EXPO_FRAME_TYPE_1FRAME:
            Gain = pGain->gain.linearGainParas;
            if (pOV2775Ctx->SensorGain.gain.linearGainParas != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor linear gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            pOV2775Ctx->SensorGain.gain.linearGainParas = pGain->gain.linearGainParas;
            TRACE(OV2775_INFO, "%s set linear gain %d\n", __func__,pGain->gain.linearGainParas);
            break;
        case ISI_EXPO_FRAME_TYPE_2FRAMES:
            if (pGain->gain.dualGainParas.dualGain > pGain->gain.dualGainParas.dualSGain * 16) {
                pGain->gain.dualGainParas.dualGain = pGain->gain.dualGainParas.dualSGain * 16;
            }
            Gain = pGain->gain.dualGainParas.dualGain;
            if (pOV2775Ctx->SensorGain.gain.dualGainParas.dualGain != Gain) {
                if (pOV2775Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &Gain);
                }
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor dual gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.dualGainParas.dualSGain;
            if (pOV2775Ctx->SensorGain.gain.dualGainParas.dualSGain != ShortGain) {
                if (pOV2775Ctx->CurMode.stitching_mode != SENSOR_STITCHING_DUAL_DCG_NOWAIT) {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                } else {
                    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &ShortGain);
                }
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor dual vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(OV2775_INFO,"%s:set gain%d short gain %d!\n", __func__,Gain,ShortGain);
            pOV2775Ctx->SensorGain.gain.dualGainParas.dualGain = Gain;
            pOV2775Ctx->SensorGain.gain.dualGainParas.dualSGain = ShortGain;
            break;
        case ISI_EXPO_FRAME_TYPE_3FRAMES:
            if ( pGain->gain.triGainParas.triLGain > pGain->gain.triGainParas.triGain * 16) {
                pGain->gain.triGainParas.triLGain = pGain->gain.triGainParas.triGain * 16;
            }
            LongGain = pGain->gain.triGainParas.triLGain;
            if (pOV2775Ctx->SensorGain.gain.triGainParas.triLGain != LongGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_LONG_GAIN, &LongGain);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            Gain = pGain->gain.triGainParas.triGain;
            if (pOV2775Ctx->SensorGain.gain.triGainParas.triGain != Gain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_GAIN, &Gain);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor tri gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }

            ShortGain = pGain->gain.triGainParas.triSGain;
            if (pOV2775Ctx->SensorGain.gain.triGainParas.triSGain != ShortGain) {
                ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_VSGAIN, &ShortGain);
                if (ret != 0) {
                    TRACE(OV2775_ERROR,"%s:set sensor tri vs gain error!\n", __func__);
                    return RET_FAILURE;
                }
            }
            TRACE(OV2775_INFO,"%s:set long gain %d gain%d short gain %d!\n", __func__, LongGain, Gain, ShortGain);
            pOV2775Ctx->SensorGain.gain.triGainParas.triLGain = LongGain;
            pOV2775Ctx->SensorGain.gain.triGainParas.triGain = Gain;
            pOV2775Ctx->SensorGain.gain.triGainParas.triSGain = ShortGain;
            break;
        default:
            return RET_FAILURE;
            break;
    }

    TRACE(OV2775_INFO, "%s (exit)\n", __func__);

    return RET_SUCCESS;
}


static RESULT OV2775_IsiGetSensorFpsIss(IsiSensorHandle_t handle, uint32_t * pfps)
{
    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    if (pfps == NULL)
        return RET_NULL_POINTER;

    *pfps = pOV2775Ctx->CurMode.ae_info.cur_fps;

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetSensorFpsIss(IsiSensorHandle_t handle, uint32_t fps)
{
    int ret = 0;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_FPS, &fps);
    if (ret != 0) {
        TRACE(OV2775_ERROR,"%s:set sensor fps error!\n", __func__);
        return RET_FAILURE;
    }
    struct vvcam_mode_info_s SensorMode;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_SENSOR_MODE, &SensorMode);
    if (ret != 0) {
        TRACE(OV2775_ERROR,"%s:get sensor mode error!\n", __func__);
        return RET_FAILURE;
    }
    memcpy(&pOV2775Ctx->CurMode, &SensorMode, sizeof(struct vvcam_mode_info_s));
    OV2775_UpdateIsiAEInfo(handle);

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
static RESULT OV2775_IsiSetSensorAfpsLimitsIss(IsiSensorHandle_t handle, uint32_t minAfps)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    if ((minAfps > pOV2775Ctx->CurMode.ae_info.max_fps) ||
        (minAfps < pOV2775Ctx->CurMode.ae_info.min_fps))
        return RET_FAILURE;
    pOV2775Ctx->minAfps = minAfps;
    pOV2775Ctx->CurMode.ae_info.min_afps = minAfps;

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetSensorIspStatusIss(IsiSensorHandle_t handle,
                               IsiSensorIspStatus_t *pSensorIspStatus)
{
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    if (pOV2775Ctx->CurMode.hdr_mode == SENSOR_MODE_HDR_NATIVE) {
        pSensorIspStatus->useSensorAWB = true;
        pSensorIspStatus->useSensorBLC = true;
    } else {
        pSensorIspStatus->useSensorAWB = false;
        pSensorIspStatus->useSensorBLC = false;
    }

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}
#ifndef ISI_LITE
static RESULT OV2775_IsiSensorSetBlcIss(IsiSensorHandle_t handle, IsiSensorBlc_t * pBlc)
{
    int32_t ret = 0;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pBlc == NULL)
        return RET_NULL_POINTER;

    struct sensor_blc_s SensorBlc;
    SensorBlc.red = pBlc->red;
    SensorBlc.gb = pBlc->gb;
    SensorBlc.gr = pBlc->gr;
    SensorBlc.blue = pBlc->blue;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_BLC, &SensorBlc);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorSetWBIss(IsiSensorHandle_t handle, IsiSensorWB_t *pWb)
{
    int32_t ret = 0;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pWb == NULL)
        return RET_NULL_POINTER;

    struct sensor_white_balance_s SensorWb;
    SensorWb.r_gain = pWb->r_gain;
    SensorWb.gr_gain = pWb->gr_gain;
    SensorWb.gb_gain = pWb->gb_gain;
    SensorWb.b_gain = pWb->b_gain;
    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_WB, &SensorWb);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: set wb error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorGetExpandCurveIss(IsiSensorHandle_t handle, IsiSensorExpandCurve_t *pExpandCurve)
{
    int32_t ret = 0;

    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    if (pExpandCurve == NULL)
        return RET_NULL_POINTER;

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_G_EXPAND_CURVE, pExpandCurve);
    if (ret != 0) {
        TRACE(OV2775_ERROR, "%s: get  expand cure error\n", __func__);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSensorGetCompressCurveIss(IsiSensorHandle_t handle, IsiSensorCompressCurve_t *pCompressCurve)
{
    int i = 0;
    TRACE(OV2775_INFO, "%s: (enter)\n", __func__);

    if (pCompressCurve == NULL)
        return RET_NULL_POINTER;

    if ((pCompressCurve->x_bit == 16) && (pCompressCurve->y_bit == 12)) {
        uint8_t compress_px[64] = {10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
					10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10};
        uint32_t compress_y_data[65] = {
                    0   ,390 ,670 ,894 ,1096,1254,1378,1493,1592,1683,1769,1849,1926,1999,2069,2136,
                    2200,2262,2322,2380,2436,2491,2544,2596,2646,2696,2744,2791,2837,2882,2926,2969,
                    3012,3054,3095,3135,3175,3214,3252,3290,3327,3364,3400,3436,3471,3506,3540,3574,
                    3608,3641,3674,3706,3738,3769,3801,3832,3862,3892,3922,3952,3981,4010,4039,4068,4095};
        pCompressCurve->compress_x_data[0] = 0;
        pCompressCurve->compress_y_data[0] = 0;
        for (i= 1; i < 65; i++) {
            pCompressCurve->compress_px[i-1] = compress_px[i-1];
            pCompressCurve->compress_x_data[i] = pCompressCurve->compress_x_data[i-1] + (1 << compress_px[i-1]);
            pCompressCurve->compress_y_data[i] = compress_y_data[i];
        }
    } else if ((pCompressCurve->x_bit == 20) && (pCompressCurve->y_bit == 12)) {
        uint8_t compress_px[64] = {
                    14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
					14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
					14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
					14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,};
        uint32_t compress_y_data[65] = {
                    0   ,390 ,670 ,894 ,1096,1254,1378,1493,1592,1683,1769,1849,1926,1999,2069,2136,
                    2200,2262,2322,2380,2436,2491,2544,2596,2646,2696,2744,2791,2837,2882,2926,2969,
                    3012,3054,3095,3135,3175,3214,3252,3290,3327,3364,3400,3436,3471,3506,3540,3574,
                    3608,3641,3674,3706,3738,3769,3801,3832,3862,3892,3922,3952,3981,4010,4039,4068,4095};
        pCompressCurve->compress_x_data[0] = 0;
        pCompressCurve->compress_y_data[0] = 0;
        for (i= 1; i < 65; i++) {
            pCompressCurve->compress_px[i-1] = compress_px[i-1];
            pCompressCurve->compress_x_data[i] = pCompressCurve->compress_x_data[i-1] + (1 << compress_px[i-1]);
            pCompressCurve->compress_y_data[i] = compress_y_data[i];
        }
    } else {
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetTestPatternIss(IsiSensorHandle_t handle,
                                       IsiSensorTpgMode_e  tpgMode)
{
    int32_t ret = 0;

    TRACE( OV2775_INFO, "%s (enter)\n", __func__);

    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;
    HalContext_t *pHalCtx = (HalContext_t *) pOV2775Ctx->IsiCtx.HalHandle;

    struct sensor_test_pattern_s TestPattern;
    if (tpgMode == ISI_TPG_DISABLE) {
        TestPattern.enable = 0;
        TestPattern.pattern = 0;
    } else {
        TestPattern.enable = 1;
        TestPattern.pattern = (uint32_t)tpgMode - 1;
    }

    ret = ioctl(pHalCtx->sensor_fd, VVSENSORIOC_S_TEST_PATTERN, &TestPattern);
    if (ret != 0)
    {
        TRACE(OV2775_ERROR, "%s: set test pattern %d error\n", __func__, tpgMode);
        return RET_FAILURE;
    }

    TRACE(OV2775_INFO, "%s: test pattern enable[%d] mode[%d]\n", __func__, TestPattern.enable, TestPattern.pattern);

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);

    return RET_SUCCESS;
}

static RESULT OV2775_IsiFocusSetupIss(IsiSensorHandle_t handle)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiFocusReleaseIss(IsiSensorHandle_t handle)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiFocusGetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiFocusSetIss(IsiSensorHandle_t handle, IsiFocusPos_t *pPos)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetFocusCalibrateIss(IsiSensorHandle_t handle, IsiFoucsCalibAttr_t *pFocusCalib)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiGetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t *pExposure)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    if (pOV2775Ctx->AEStartExposure == 0) {
        pOV2775Ctx->AEStartExposure =
            (uint64_t)pOV2775Ctx->CurMode.ae_info.start_exposure *
            pOV2775Ctx->CurMode.ae_info.one_line_exp_time_ns / 1000;
           
    }
    *pExposure =  pOV2775Ctx->AEStartExposure;
    TRACE(OV2775_INFO, "%s:get start exposure %d\n", __func__, pOV2775Ctx->AEStartExposure);

    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}

static RESULT OV2775_IsiSetAeStartExposureIs(IsiSensorHandle_t handle, uint64_t exposure)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);
    OV2775_Context_t *pOV2775Ctx = (OV2775_Context_t *) handle;

    pOV2775Ctx->AEStartExposure = exposure;
    TRACE(OV2775_INFO, "set start exposure %d\n", __func__,pOV2775Ctx->AEStartExposure);
    TRACE(OV2775_INFO, "%s: (exit)\n", __func__);
    return RET_SUCCESS;
}
#endif

RESULT OV2775_IsiGetSensorIss(IsiSensor_t *pIsiSensor)
{
    TRACE( OV2775_INFO, "%s (enter)\n", __func__);

    if (pIsiSensor == NULL)
        return RET_NULL_POINTER;
     pIsiSensor->pszName                         = SensorName;
     pIsiSensor->pIsiSensorSetPowerIss           = OV2775_IsiSensorSetPowerIss;
     pIsiSensor->pIsiCreateSensorIss             = OV2775_IsiCreateSensorIss;
     pIsiSensor->pIsiReleaseSensorIss            = OV2775_IsiReleaseSensorIss;
     pIsiSensor->pIsiRegisterReadIss             = OV2775_IsiRegisterReadIss;
     pIsiSensor->pIsiRegisterWriteIss            = OV2775_IsiRegisterWriteIss;
     pIsiSensor->pIsiGetSensorModeIss            = OV2775_IsiGetSensorModeIss;
     pIsiSensor->pIsiSetSensorModeIss            = OV2775_IsiSetSensorModeIss;
     pIsiSensor->pIsiQuerySensorIss              = OV2775_IsiQuerySensorIss;
     pIsiSensor->pIsiGetCapsIss                  = OV2775_IsiGetCapsIss;
     pIsiSensor->pIsiSetupSensorIss              = OV2775_IsiSetupSensorIss;
     pIsiSensor->pIsiGetSensorRevisionIss        = OV2775_IsiGetSensorRevisionIss;
     pIsiSensor->pIsiCheckSensorConnectionIss    = OV2775_IsiCheckSensorConnectionIss;
     pIsiSensor->pIsiSensorSetStreamingIss       = OV2775_IsiSensorSetStreamingIss;
     pIsiSensor->pIsiGetAeInfoIss                = OV2775_IsiGetAeInfoIss;
     pIsiSensor->pIsiSetHdrRatioIss              = OV2775_IsiSetHdrRatioIss;
     pIsiSensor->pIsiGetIntegrationTimeIss       = OV2775_IsiGetIntegrationTimeIss;
     pIsiSensor->pIsiSetIntegrationTimeIss       = OV2775_IsiSetIntegrationTimeIss;
     pIsiSensor->pIsiGetGainIss                  = OV2775_IsiGetGainIss;
     pIsiSensor->pIsiSetGainIss                  = OV2775_IsiSetGainIss;
     pIsiSensor->pIsiGetSensorFpsIss             = OV2775_IsiGetSensorFpsIss;
     pIsiSensor->pIsiSetSensorFpsIss             = OV2775_IsiSetSensorFpsIss;
     pIsiSensor->pIsiSetSensorAfpsLimitsIss      = OV2775_IsiSetSensorAfpsLimitsIss;
     pIsiSensor->pIsiGetSensorIspStatusIss       = OV2775_IsiGetSensorIspStatusIss;
#ifndef ISI_LITE
    pIsiSensor->pIsiSensorSetBlcIss              = OV2775_IsiSensorSetBlcIss;
    pIsiSensor->pIsiSensorSetWBIss               = OV2775_IsiSensorSetWBIss;
    pIsiSensor->pIsiSensorGetExpandCurveIss      = OV2775_IsiSensorGetExpandCurveIss;
    pIsiSensor->pIsiSensorGetCompressCurveIss    = OV2775_IsiSensorGetCompressCurveIss;
    pIsiSensor->pIsiActivateTestPatternIss       = OV2775_IsiSetTestPatternIss;
    pIsiSensor->pIsiFocusSetupIss                = OV2775_IsiFocusSetupIss;
    pIsiSensor->pIsiFocusReleaseIss              = OV2775_IsiFocusReleaseIss;
    pIsiSensor->pIsiFocusSetIss                  = OV2775_IsiFocusSetIss;
    pIsiSensor->pIsiFocusGetIss                  = OV2775_IsiFocusGetIss;
    pIsiSensor->pIsiGetFocusCalibrateIss         = OV2775_IsiGetFocusCalibrateIss;
    pIsiSensor->pIsiSetAeStartExposureIss        = OV2775_IsiSetAeStartExposureIs;
    pIsiSensor->pIsiGetAeStartExposureIss        = OV2775_IsiGetAeStartExposureIs;
#endif
    TRACE( OV2775_INFO, "%s (exit)\n", __func__);
    return RET_SUCCESS;
}

/*****************************************************************************
* each sensor driver need declare this struct for isi load
*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig = {
    .CameraDriverID = 0x2770,
    .pIsiHalQuerySensor = OV2775_IsiHalQuerySensorIss,
    .pfIsiGetSensorIss = OV2775_IsiGetSensorIss,
};
