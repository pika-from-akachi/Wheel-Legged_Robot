/**
  ******************************************************************************
  * @file    can_rx_handler.h
  * @brief   CAN Receive Handler Header File
  ******************************************************************************
  */

#ifndef __CAN_RX_HANDLER_H__
#define __CAN_RX_HANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Enhanced CAN RX callback with complete data parsing
 * @param hcan: Pointer to CAN handle
 * @retval None
 */
void EL05_CAN_RxHandler(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_RX_HANDLER_H__ */
