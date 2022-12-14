/******************************************************************************\
|* Copyright 2010, Dream Chip Technologies GmbH. used with permission by      *|
|* VeriSilicon.                                                               *|
|* Copyright (c) <2020> by VeriSilicon Holdings Co., Ltd. ("VeriSilicon")     *|
|* All Rights Reserved.                                                       *|
|*                                                                            *|
|* The material in this file is confidential and contains trade secrets of    *|
|* of VeriSilicon.  This is proprietary information owned or licensed by      *|
|* VeriSilicon.  No part of this work may be disclosed, reproduced, copied,   *|
|* transmitted, or used in any way for any purpose, without the express       *|
|* written permission of VeriSilicon.                                         *|
|*                                                                            *|
\******************************************************************************/

/* VeriSilicon 2020 */

#ifndef __AEC_H__
#define __AEC_H__

/**
 * @file aec.h
 *
 * @brief
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup AECM Auto white Balance Module
 * @{
 *
 */
#include <ebase/types.h>
#include <common/return_codes.h>

#include <isi/isi_iss.h>
#include <isi/isi.h>

#include <cameric_drv/cameric_drv_api.h>
#include <cameric_drv/cameric_isp_drv_api.h>
#include <cameric_drv/cameric_isp_exp_drv_api.h>
#include <cameric_drv/cameric_isp_hist_drv_api.h>

#include <cam_calibdb/cam_calibdb_api.h>

#ifdef __cplusplus
extern "C"
{
#endif



typedef struct AecContext_s *AecHandle_t;       /**< handle to AEC context */
typedef struct AecContext_s *Aev2Handle_t;       /**< handle to AEC context */

typedef enum AecState_e
{
    AEC_STATE_INVALID       = 0,
    AEC_STATE_INITIALIZED   = 1,
    AEC_STATE_STOPPED       = 2,
    AEC_STATE_RUNNING       = 3,
    AEC_STATE_LOCKED        = 4,
    AEC_STATE_MAX
} AecState_t;

typedef enum AecLockStatus_e
{
    AEC_UNLOCK = 0,
    AEC_LOCK
} AecLockStatus_t;

typedef enum AecSensitivityIso_e
{
    AEC_SENSITIVITY_ISO_INVALID,
    AEC_SENSITIVITY_100_ISO    = 100,
    AEC_SENSITIVITY_200_ISO    = 200,
    AEC_SENSITIVITY_400_ISO    = 400,
    AEC_SENSITIVITY_800_ISO    = 800,
    AEC_SENSITIVITY_1600_ISO   = 1600,
    AEC_SENSITIVITY_ISO_MAX
} AecSensitivityIso_t;

typedef enum AecSensitivityGain_e
{
    AEC_SENSITIVITY_GAIN_INVALID,
    AEC_SENSITIVITY_100_GAIN   = 1,
    AEC_SENSITIVITY_200_GAIN   = 2,
    AEC_SENSITIVITY_400_GAIN   = 4,
    AEC_SENSITIVITY_800_GAIN   = 8,
    AEC_SENSITIVITY_1600_GAIN  = 16,
    AEC_SENSITIVITY_GAIN_MAX
} AecSensitivityGain_t;

/*****************************************************************************/
/**
 *          AecSemMode_t
 *
 * @brief   mode type of AEC Scene Evaluation
 *
 */
/*****************************************************************************/
typedef enum AecSemMode_e
{
    AEC_SCENE_EVALUATION_INVALID    = 0,        /* invalid (only used for initialization) */
    AEC_SCENE_EVALUATION_DISABLED   = 1,        /* Scene Evaluation disabled (fix setpoint) */
    AEC_SCENE_EVALUATION_FIX        = 2,        /* Scene Evaluation fix (static ROI) */
    AEC_SCENE_EVALUATION_ADAPTIVE   = 3,        /* Scene Evaluation adaptive (ROI caluclated by Scene Evaluation */
    AEC_SCENE_EVALUATION_MAX
} AecSemMode_t;


/*****************************************************************************/
/**
 *          AecEcmMode_t
 *
 * @brief   mode type of AEC Exposure Conversion
 *
 */
/*****************************************************************************/
typedef enum AecEcmMode_e
{
    AEC_EXPOSURE_CONVERSION_INVALID = 0,        /* invalid (only used for initialization) */
    AEC_EXPOSURE_CONVERSION_LINEAR  = 1,        /* Exposure Conversion uses a linear function (eq. 38) */
    AEC_EXPOSURE_CONVERSION_MAX
} AecEcmMode_t;


/*****************************************************************************/
/**
 *          AecEcmFlickerPeriod_t
 *
 * @brief   flicker period types for the AEC algorithm
 *
 */
/*****************************************************************************/
typedef enum AecEcmFlickerPeriod_e
{
    AEC_EXPOSURE_CONVERSION_FLICKER_OFF   = 0x00,
    AEC_EXPOSURE_CONVERSION_FLICKER_100HZ = 0x01,
    AEC_EXPOSURE_CONVERSION_FLICKER_120HZ = 0x02
} AecEcmFlickerPeriod_t;


/*****************************************************************************/
/**
 *
 * @brief   callback function type for AEC/AFPS resolution change request
 *
 */
/*****************************************************************************/
typedef void (AfpsResChangeCb_t)
(
    void        *pPrivateContext,               /**< reference to user context as handed in via AecInstanceConfig_t */
    uint32_t    NewResolution                   /**< new resolution to switch to */
);

/*****************************************************************************/
/**
 * @brief   AEC Module instance configuration structure
 *
 *****************************************************************************/
typedef struct AecInstanceConfig_s
{
    AfpsResChangeCb_t   *pResChangeCbFunc;      /**< callback function to trigger resolution change */
    void                *pResChangeCbContext;   /**< reference to context to pass to callback */

    AecHandle_t         hAec;                   /**< handle returned by AecInit() */
    Aev2Handle_t        hAev2;                   /**< handle returned by AecInit() */
} AecInstanceConfig_t;




/*****************************************************************************/
/**
 *          AecConfig_t
 *
 * @brief   AEC Module configuration structure; used for re-configuration as well
 *
 *****************************************************************************/
typedef struct AecConfig_s
{
    IsiSensorHandle_t       hSensor;                /**< sensor handle; not evaluated during re-configuration */
    IsiSensorHandle_t       hSubSensor;             /**< sensor handle; not evaluated during re-configuration */

    CamCalibDbHandle_t      hCamCalibDb;            /**< calibration database handle */

    AecSemMode_t            SemMode;                /**< scene evaluation mode */

    float                   SetPoint;               /**< set point to hit by the ae control system */
    float                   ClmTolerance;

    float                   DampOver;
    float                   DampUnder;

    bool_t                  AfpsEnabled;            /**< AFPS mode control */
    float                   AfpsMaxGain;            /**< AFPS max gain */

    AecEcmFlickerPeriod_t   EcmFlickerSelect;       /**< flicker period selection */
//    float                   EcmT0fac;               /**< start of flicker avoidance as multiple of flicker period: EcmT0 = EcmT0fac * EcmTflicker*/
//    float                   EcmA0;                  /**< linear: slope of gain */
    uint8_t                 Weight[CAMERIC_ISP_EXP_GRID_ITEMS];
} AecConfig_t;



/*****************************************************************************/
/**
 * @brief   This function creates and initializes an AEC instance.
 *
 * @param   pInstConfig     pointer to the instance configuration
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT AecInit
(
    AecInstanceConfig_t *pInstConfig
);



/*****************************************************************************/
/**
 * @brief   The function releases/frees the given AEC instance.
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecRelease
(
    AecHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This function configures the AEC instance.
 *
 * @param   handle      AEC instance handle
 * @param   pConfig     pointer to configuration structure
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT AecConfigure
(
    AecHandle_t handle,
    AecConfig_t *pConfig
);



/*****************************************************************************/
/**
 * @brief   This function re-configures the AEC instance, e.g. after a
 *          resolution change
 *
 * @param   handle              AEC instance handle
 * @param   pConfig             pointer to configuration structure
 * @param   pNumFramesToSkip    reference of storage for number of frames that have to be skipped
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT AecReConfigure
(
    AecHandle_t handle,
    AecConfig_t *pConfig,
    uint32_t    *pNumFramesToSkip
);



/*****************************************************************************/
/**
 * @brief   This function is set or clear aec lock
 *
 * @param   handle              AEC instance handle
 * @param   LockStatus          AEC_LOCK or AEC_UNLOCK
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
RESULT AecSetLockStatus(AecHandle_t handle, AecLockStatus_t LockStatus);



/*****************************************************************************/
/**
 * @brief   get aec lock status
 *
 * @param   handle              AEC instance handle
 * @param   pLockStatus         point to AEC LOCK STATUS
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT AecGetLockStatus(AecHandle_t handle, AecLockStatus_t *pLockStatus);




/*****************************************************************************/
/**
 * @brief   This functions processes the Scene Evaluation Module (SEM) of the
 *          given AEC instance for the current frame.
 *
 * @param   handle      AEC instance handle
 * @param   luma        5x5 array of the measured mean luma values
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecSemExecute
(
    AecHandle_t         handle,
    CamerIcMeanLuma_t   luma,
    uint8_t             ext_bit
);



/*****************************************************************************/
/**
 * @brief   This function processes the Control Loop Module (CLM) of hdr ratio
 *              from hist stats.
 *
 * @param   handle      AEC instance handle
 * @param   hdr_ratio   hdr ratios
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecClmHdrRatio
(
    AecHandle_t         handle,
    float *             hdr_ratio
);

/*****************************************************************************/
/**
 * @brief   This function update the Histogram ColorConversionRange mode of Control Loop Module (CLM).
 *
 * @param   handle          AEC instance handle
 * @param   HistogramRange  Luminance ColorConversionRange of Histogram
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecClmUpdateHistogramRange
(
    AecHandle_t         handle,
    CamerIcColorConversionRange_t HistogramRange
);

/*****************************************************************************/
/**
 * @brief   This function processes the Control Loop Module (CLM) of the
 *          given AEC instance for the current frame.
 *
 * @param   handle      AEC instance handle
 * @param   bins        histogramm
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecClmExecute
(
    AecHandle_t         handle,
    CamerIcHistBins_t   bins,
    float *hdr_ratio
);



/*****************************************************************************/
/**
 * @brief   This function returns BOOL_TRUE if the AEC is settled.
 *
 * @param   handle      AEC instance handle
 * @param   pSettled    pointer to settled value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecSettled
(
    AecHandle_t handle,
    bool_t      *pSettled
);

RESULT AecGetAeDelay(AecHandle_t handle, uint32_t * pIntTimeDelay, uint32_t *pGainDelay);
RESULT AecSetHdrRatio(AecHandle_t handle, float HdrRatio);
RESULT AecSetIntTime(AecHandle_t handle);
RESULT AecSetGain(AecHandle_t handle);
AecState_t AecGetRuningStatus(AecHandle_t handle);
RESULT AecRefreshEffctiveParams(AecHandle_t handle);

/*****************************************************************************/
/**
 * @brief   This function returns the currently sensitivity(ISO).
 *
 * @param   handle      AEC instance handle
 * @pram    pIso        pointer to ISO value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetSensitivity
(
    AecHandle_t               handle,
    AecSensitivityIso_t       *pIso
);

RESULT AecSensitivityRange
(
    AecHandle_t               handle,
    AecSensitivityIso_t       *minIso,
    AecSensitivityIso_t       *maxIso
);

/*****************************************************************************/
/**
 * @brief   This function set the currently sensitivity(ISO).
 *
 * @param   handle      AEC instance handle
 * @pram    isoValue    ISO value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecSetSensitivity
(
    AecHandle_t               handle,
    AecSensitivityIso_t       isoValue
);

/*****************************************************************************/
/**
 * @brief   This function returns the currently calculated Gain.
 *
 * @param   handle      AEC instance handle
 * @pram    pGain       pointer to gain value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetCurrentGain
(
    AecHandle_t handle,
    float       *pGain
);

RESULT AecGetVSCurrentGain
(
    AecHandle_t handle,
    float       *pGain
);

RESULT AecGetCurrentLongGain
(
    AecHandle_t handle,
    float       *pGain
);


/*****************************************************************************/
/**
 * @brief   This function returns the currently calculated IntegrationTime.
 *
 * @param   handle              AEC instance handle
 * qparam   pIntegrationTime    pointer to integration time value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetCurrentIntegrationTime
(
    AecHandle_t handle,
    float       *pIntegrationTime
);

RESULT AecGetVSCurrentIntegrationTime
(
    AecHandle_t handle,
    float       *pIntegrationTime
);

RESULT AecGetCurrentLongIntegrationTime
(
    AecHandle_t handle,
    float       *pIntegrationTime
);

/*****************************************************************************/
/**
 * @brief   This function returns the current histogram.
 *
 * @param   handle              AEC instance handle
 * qparam   pHistogram          pointer to the histogram bins
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetCurrentHistogram
(
    AecHandle_t         handle,
    CamerIcHistBins_t   *pHistogram
);



/*****************************************************************************/
/**
 * @brief   This function returns the current luminance grid.
 *
 * @param   handle              AEC instance handle
 * qparam   pHistogram          pointer to the luminance grid
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetCurrentLuminance
(
    AecHandle_t         handle,
    CamerIcMeanLuma_t   *pLuma
);



/*****************************************************************************/
/**
 * @brief   This function returns the current object region.
 *
 * @param   handle              AEC instance handle
 * qparam   pHistogram          pointer to the object region
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 *
 *****************************************************************************/
RESULT AecGetCurrentObjectRegion
(
    AecHandle_t         handle,
    CamerIcMeanLuma_t   *pObjectRegion
);



/*****************************************************************************/
/**
 * @brief   This function returns the current configuration.
 *
 * @param   handle      AEC instance handle
 * @param   pConfig     reference of configuration structure to be filled with
 *                      the current configuration
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT AecGetCurrentConfig
(
    AecHandle_t handle,
    AecConfig_t *pConfig
);



/*****************************************************************************/
/**
 * @brief   This function returns the status of the AEC module.
 *
 * @param   handle      AEC instance handle
 * @param   pRunning    pointer to state value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecStatus
(
    AecHandle_t     handle,
    bool_t          *pRunning,
    AecSemMode_t    *pMode,
    float           *pSetPoint,
    float           *pClmTolerance,
    float           *pDampOver,
    float           *pDampUnder
);



/*****************************************************************************/
/**
 * @brief   This function starts the AEC instance.
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 * @retval  RET_OUTOFRANGE
 *
 *****************************************************************************/
RESULT AecStart
(
    AecHandle_t handle
);


RESULT Aev2Start
(
    Aev2Handle_t handle
);
/*****************************************************************************/
/**
 * @brief
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecStop
(
    AecHandle_t handle
);



/*****************************************************************************/
/**
 * @brief
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecReset
(
    AecHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This functions tries to lock the AEC instance.
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecTryLock
(
    AecHandle_t handle
);


/*****************************************************************************/
/**
 * @brief   This functions unlocks the AEC instance.
 *
 * @param   handle  AEC instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AecUnLock
(
    AecHandle_t handle
);


float Aev2MeanLuma
(
    Aev2Handle_t         handle,
    unsigned char *Expv2
);


RESULT Aev2ClmExecute
(
    Aev2Handle_t         handle,
    unsigned char        *aev2,
    float                *hdr_ratio
);

RESULT Aev2Init
(
    AecInstanceConfig_t *pInstConfig
);

#ifdef __cplusplus
}
#endif

/* @} AECM */


#endif /* __AEC_H__*/
