
#include "imu_interrupt.h"

#include "BMI088driver.h"
#include "imu_pwm.h"
#include "main.h"
#include "pid.h"
#include "bsp_can.h"

#define CAN_ID 0x100
#define CAMERA_TRIGGER_PRESCALER 5

#define TEMPERATURE_DESIRED 45.0f
#define TEMPERATURE_PID_KP 1600.0f         // kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f            // ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f            // kd of temperature control PID
#define TEMPERATURE_PID_MAX_OUT 4998.0f    // max out of temperature control PID
#define TEMPERATURE_PID_MAX_I_OUT 4400.0f  // max I out of temperature control PID

extern SPI_HandleTypeDef hspi1;
extern CAN_HandleTypeDef hcan;

uint8_t imu_start_flag = 0;
uint8_t camera_trigger_count = CAMERA_TRIGGER_PRESCALER;

const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static CAN_TxHeaderTypeDef can_header;
static uint8_t can_data[8];

uint8_t camera_start_flag = 0;
uint8_t trigger_start_delay = 0;

void imu_interrupt_init(void) {
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_I_OUT);
    while (BMI088_init()) {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
        HAL_Delay(100);
    }
    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
    imu_start_flag = 1;
    can_header.IDE = CAN_ID_STD;
    can_header.RTR = CAN_RTR_DATA;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_GRYO_Pin) {
        if (imu_start_flag) {
          uint32_t send_mail_box;
          can_header.StdId = CAN_ID;
          can_header.DLC = 0x08;
          get_BMI088_gyro_raw(can_data);
          get_BMI088_temperate_raw(&can_data[6]);
          HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &send_mail_box);
          fp32 temp = BMI088_temperature_read_over(&can_data[6]);
          PID_calc(&imu_temp_pid, temp, TEMPERATURE_DESIRED);
          if (imu_temp_pid.out < 0.0f)
            imu_temp_pid.out = 0.0f;
          uint16_t tempPWM = (uint16_t) imu_temp_pid.out;
          imu_pwm_set(tempPWM);
        }
    } else if (GPIO_Pin == INT1_ACCEL_Pin) {
        if (imu_start_flag) {
          can_header.StdId = CAN_ID + 1;
          uint32_t send_mail_box;
          can_header.DLC = 0x07;
          get_BMI088_accel_raw(can_data);
          can_data[6] = 0;
          if (trigger_start_delay != 1) trigger_start_delay--;
          if(camera_start_flag) can_data[6] |= 2;
          if (camera_trigger_count == 1) {
            camera_trigger_count = CAMERA_TRIGGER_PRESCALER;
            if (camera_start_flag  && trigger_start_delay == 1) {
              HAL_GPIO_WritePin(CAM_GPIO_Port, CAM_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
                can_data[6] |= 1;
            }else{
                HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
          } else {
            HAL_GPIO_WritePin(CAM_GPIO_Port, CAM_Pin, GPIO_PIN_RESET);
            camera_trigger_count--;
          }
          HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &send_mail_box);
        }
    }
}