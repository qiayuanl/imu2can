#include "imu_pwm.h"

#include "main.h"

void imu_pwm_set(uint16_t pwm) {
    TIM2->CCR2 = (pwm);
}
