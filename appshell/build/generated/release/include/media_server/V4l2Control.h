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

#ifndef DEVELOPER_MEDIACONTROL_SERVER_V4L2CONTROL_H_
#define DEVELOPER_MEDIACONTROL_SERVER_V4L2CONTROL_H_

#include "IMediaModule.h"

class V4l2Control : public IMediaModule {
    DECLARE_DYNAMIC_CLASS()
 public:
    V4l2Control();
    virtual ~V4l2Control();
    bool open(int id, Json::Value& node) override;
    bool close() override;
    virtual bool run(uint32_t nSinkIndex) override;
    virtual const char* getClassName() override { return "V4l2Control"; }
 private:
    int fd = -1;
};

#endif  // DEVELOPER_MEDIACONTROL_SERVER_V4L2CONTROL_H_

