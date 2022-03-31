#include "bsp_can.h"

#include "main.h"

CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data;
extern CAN_HandleTypeDef hcan;
extern uint8_t camera_start_flag;
//extern CAN_HandleTypeDef hcan2;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance == CAN1) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, &can_rx_data);
    if (can_rx_data) {
      camera_start_flag = 1;
    } else {
      camera_start_flag = 0;
    }
  }
}

void can_init(void) {
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = (0x102) << 5;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0xFFF;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
