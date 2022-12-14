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

#include "test_base.h"
#include <assert.h>
#include <stdio.h>

#define MAX_LEN 256

int CL_LoadCaseList(const char *pFilename, char ***pppCaseList,
                    int *pCaseCount) {
    FILE *pF;
    char caseName[MAX_LEN];
    char **ppList = NULL;
    int num = 0;

    int ret = 0;
    int len = 0;

    if (!pppCaseList || !pCaseCount) {
        return -EINVAL;
    }

    assert(pFilename);

    pF = fopen(pFilename, "r");

    if (!pF) {
        return -ENODEV;
    }

    num = 0;

    while (fgets(caseName, MAX_LEN, pF)) {
        if ((caseName[0] == '\n') || (caseName[0] == '#'))
            continue;

        num++;
    }

    ppList = malloc(num * sizeof(char *));

    fseek(pF, 0, SEEK_SET);

    num = 0;

    while (fgets(caseName, MAX_LEN, pF)) {
        if (caseName[0] == '\n' || caseName[0] == '#' || caseName[0] == '\r') {
            continue;
        }
        len = strlen(caseName);

        if (caseName[len - 1] == '\n') {
            caseName[len - 1] = '\0';
        }
        //Dos format case txt supported
        if (caseName[len - 2] == '\r') {
            caseName[len - 2] = '\0';
        }

        ppList[num] = malloc(MAX_LEN);

        if (ppList[num]) {
            strncpy(ppList[num], caseName, MAX_LEN);
        }

        num++;
    }

    *pppCaseList = ppList;
    *pCaseCount = num;

    fclose(pF);

    return ret;
}

void CL_FreeCaseList(char **pppCaseList, int pCaseCount) {
    int i;

    if (!pppCaseList || !pCaseCount)
        return;

    for (i = 0; i < pCaseCount; i++) {
        if (pppCaseList[i]) {
            free(pppCaseList[i]);
        }
    }

    free(pppCaseList);

    return;
}
