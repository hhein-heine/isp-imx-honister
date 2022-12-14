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

/**
 * @file dom_ctrl_vidplay_api.h
 *
 * @brief
 *   Definition of DOM Ctrl Videoplayer API.
 *
 *****************************************************************************/
/**
 * @page dom_ctrl_page DOM Ctrl
 * The Display Output Module Videoplayer displays image buffers in an X11 window.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref dom_ctrl_api
 * - @ref dom_ctrl_common
 * - @ref dom_ctrl
 *
 * @defgroup dom_ctrl_vidplay_api DOM Ctrl Videoplayer API
 * @{
 *
 */

#ifndef __DOM_CTRL_VIDPLAY_API_H__
#define __DOM_CTRL_VIDPLAY_API_H__
#include <types.h>
#include <cam_device_buf_defs.h>
#include "return_codes.h"

#include <appshell_oslayer/oslayer.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct domCtrlVidplayHandle_s *domCtrlVidplayHandle_t;

typedef struct domCtrlVidplayConfig_s
{
    void                    *hParent;       //!< IN: Anonymous handle to window parent; NULL if stand alone window. Real handle type is implementation dependent; user must assure correctness at design time.
    int32_t                 posX;           //!< IN: Window, Widget, Control, ... top left corner relative to parent or display area.
    int32_t                 posY;           //!< IN: Window, Widget, Control, ... top left corner relative to parent or display area.
    uint32_t                width;          //!< IN: Window, Widget, Control, ... dimension.
    uint32_t                height;         //!< IN: Window, Widget, Control, ... dimension.

    domCtrlVidplayHandle_t  domCtrlVidplayHandle;   //!< OUT: Handle to created dom control videoplayer context, set by @ref domCtrlVidplayInit if successfull, undefined otherwise.
} domCtrlVidplayConfig_t;

extern RESULT domCtrlVidplayInit
(
    domCtrlVidplayConfig_t *pConfig                 //!< Reference to configuration structure.
);

extern RESULT domCtrlVidplayShutDown
(
    domCtrlVidplayHandle_t  domCtrlVidplayHandle    //!< Handle to dom control videoplayer context as returned by @ref domCtrlVidplayInit.
);

extern RESULT domCtrlVidplayDisplay
(
    domCtrlVidplayHandle_t  domCtrlVidplayHandle,   //!< Handle to dom control videoplayer context as returned by @ref domCtrlVidplayInit.
    PicBufMetaData_t        *pPicBufMetaData        //!< The meta data describing the image buffer to display (includes the data pointers as well...).
);

extern RESULT domCtrlVidplayClear
(
    domCtrlVidplayHandle_t  domCtrlVidplayHandle    //!< Handle to dom control videoplayer context as returned by @ref domCtrlVidplayInit.
);

extern RESULT domCtrlVidplayShow
(
    domCtrlVidplayHandle_t  domCtrlVidplayHandle,   //!< Handle to dom control videoplayer context as returned by @ref domCtrlVidplayInit.
    bool_t                  show
);

extern RESULT domCtrlVidplaySetOverlayText
(
    domCtrlVidplayHandle_t  domCtrlVidplayHandle,   //!< Handle to dom control videoplayer context as returned by @ref domCtrlVidplayInit.
    char                    *szOverlayText          //!< Pointer to zero terminated string holding performance data to display; may be NULL.
);

/* @} dom_ctrl_vidplay_api */

#ifdef __cplusplus
}
#endif

#endif /* __DOM_CTRL_VIDPLAY_API_H__ */
