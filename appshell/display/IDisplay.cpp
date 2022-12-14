/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 VeriSilicon Holdings Co., Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************/
#include "IDisplay.h"
#include "stdlib.h"

#ifdef WITH_DRM
#include "DrmDisplay.h"
#endif
#include "FBDisplay.h"
#include "FileSink.h"
#include "WlDisplay.h"

IDisplay* IDisplay::createObject(int type) {
    switch (type) {
    case VIV_DISPLAY_TYPE_EMPTY:
        return new EmptySink();
#ifdef WITH_DRM
    case VIV_DISPLAY_TYPE_DRM:
        return new DrmDisplay();
#endif
    case VIV_DISPLAY_TYPE_FB:
        return new FBDisplay();
    case VIV_DISPLAY_TYPE_FILE:
        return new FileSink();
#ifdef WAYLAND_SUPPORT
    case VIV_DISPLAY_TYPE_WL:
        return new WlDisplay();
#endif
    default:
        return NULL;
    }
    return NULL;
}