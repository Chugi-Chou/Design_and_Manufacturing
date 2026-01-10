//
// Created by zhouzhi on 2026/1/10.
//

#include "AppMain.h"
#include "Motor.h"
#include "can.h"

PID s_pid(0.025f, 0.0002f, 0.005f, 0.2f, 10000.0f, 2000.0f, 0.5f);
PID p_pid(10.0f, 0.01f, 0.0f, 0.2f, 500.0f, 0.0f, 0.0f);

MotorM2006 motor_task(p_pid, s_pid);
MotorM2006 motor_always(p_pid, s_pid);

void CAN_Send(int16_t c1, int16_t c2) {
    CAN_TxHeaderTypeDef hdr;
    uint32_t box;
    uint8_t d[8] = { (uint8_t)(c1>>8), (uint8_t)c1, (uint8_t)(c2>>8), (uint8_t)c2, 0,0,0,0 };
    hdr.StdId = 0x200; hdr.IDE = CAN_ID_STD; hdr.RTR = CAN_RTR_DATA; hdr.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &hdr, d, &box);
}

extern "C" {
    void App_Init(void) {
        CAN_FilterTypeDef f = {0, 0, 0, 0, 0, CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE};
        HAL_CAN_ConfigFilter(&hcan1, &f);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

        extern TIM_HandleTypeDef htim6;
        HAL_TIM_Base_Start_IT(&htim6);

        motor_always.SetTarget(SPEED_MODE, 1000.0f);
    }

    void App_Task_1ms(void) {
        static uint16_t btn_filter = 0; //防抖滤波
        static bool is_waiting_for_finish = false; //状态锁
        bool raw_pin = (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) == GPIO_PIN_RESET);

        if (!is_waiting_for_finish) {
            if (raw_pin) {
                btn_filter++;
                if (btn_filter >= 20) {
                    motor_task.SetTarget(POSITION_MODE, motor_task.GetCurrentAngle() + 360.0f);
                    is_waiting_for_finish = true;
                    btn_filter = 0;
                }
            } else {
                btn_filter = 0;
            }
        } else {

            float error = motor_task.GetCurrentAngle() - (motor_task.GetCurrentAngle());
            if (motor_task.IsTargetReached()) {
                if (!raw_pin) {
                    is_waiting_for_finish = false;
                }
            }
        }

        CAN_Send(motor_task.ExecuteControl(), motor_always.ExecuteControl());
    }

    void App_CAN_Callback(uint32_t std_id, uint8_t* data) {
        if (std_id == 0x201) motor_task.UpdateFeedback(data);
        else if (std_id == 0x202) motor_always.UpdateFeedback(data);
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if (htim->Instance == TIM6) {
            App_Task_1ms();
        }
    }

    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
        CAN_RxHeaderTypeDef hdr;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
            App_CAN_Callback(hdr.StdId, data);
        }
    }

}
