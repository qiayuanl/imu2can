
#include "imu_interrupt.h"

#include "BMI088driver.h"
#include "imu_pwm.h"
#include "main.h"
#include "pid.h"

#define TEMPERATURE_DESIRED 45.0f
#define TEMPERATURE_PID_KP 1600.0f         // kp of temperature control PID
#define TEMPERATURE_PID_KI 0.2f            // ki of temperature control PID
#define TEMPERATURE_PID_KD 0.0f            // kd of temperature control PID
#define TEMPERATURE_PID_MAX_OUT 4998.0f    // max out of temperature control PID
#define TEMPERATURE_PID_MAX_I_OUT 4400.0f  // max I out of temperature control PID

extern SPI_HandleTypeDef hspi1;
extern CAN_HandleTypeDef hcan1;

volatile uint8_t imu_start_flag = 0;

fp32 gyro[3], accel[3], temp;
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
pid_type_def imu_temp_pid;

static CAN_TxHeaderTypeDef can_header;
static uint8_t can_data[8];

void imu_interrupt_init(void) {
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_I_OUT);
    while (BMI088_init()) {
    }
    // set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
    imu_start_flag = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        if (imu_start_flag) {
            BMI088_read(gyro, accel, &temp);
            PID_calc(&imu_temp_pid, temp, TEMPERATURE_DESIRED);
            if (imu_temp_pid.out < 0.0f)
                imu_temp_pid.out = 0.0f;
            uint16_t tempPWM = (uint16_t)imu_temp_pid.out;
            imu_pwm_set(tempPWM);
            uint32_t send_mail_box;
            can_header.StdId = 0x300;
            can_header.IDE = CAN_ID_STD;
            can_header.RTR = CAN_RTR_DATA;
            can_header.DLC = 0x08;
            can_data[0] = 0xFF;
            can_data[1] = 0x00;
            can_data[2] = 0xFF;
            can_data[3] = 0x00;
            can_data[4] = 0xFF;
            can_data[5] = 0x00;
            can_data[6] = 0xFF;
            can_data[7] = 0x00;
            HAL_CAN_AddTxMessage(&hcan1, &can_header, can_data, &send_mail_box);
        }
    } else if (GPIO_Pin == INT1_GRYO_Pin) {
    }
}
