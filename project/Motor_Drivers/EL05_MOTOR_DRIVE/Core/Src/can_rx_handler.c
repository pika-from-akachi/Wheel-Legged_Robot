/**
  ******************************************************************************
  * @file    can_rx_handler.c
  * @brief   CAN RX interrupt callback - delegates to EL05 driver
  ******************************************************************************
  */

#include "el05_motor.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    extern volatile uint32_t g_dbg_cb_fired;
    g_dbg_cb_fired++;
    EL05_CAN_RxCallback(hcan);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    EL05_CAN_RxCallback(hcan);
}
