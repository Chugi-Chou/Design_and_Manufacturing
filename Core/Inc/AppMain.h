//
// Created by zhouzhi on 2026/1/10.
//

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

    void App_Init(void);
    void App_Task_1ms(void);
    void App_CAN_Callback(uint32_t std_id, uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif
