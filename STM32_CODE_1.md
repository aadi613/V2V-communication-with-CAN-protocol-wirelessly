**The code for first STM32 board to interface with different sensors and wirelessly interact with other stm32 board**
cpp
```
// main.c - Board 1 TX side
// Uses: NRF24L01 lib, HC-SR04, MPU-6050, button

#include "main.h"
#include "nrf24l01.h"
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>

// Data packet structure — same on both boards
typedef struct {
    float distance_m;
    float accel_g;
    uint8_t brake_alert;
} VehicleData_t;

// ── HC-SR04 ──────────────────────────────────────────
float HC_SR04_GetDistance(void) {
    // Trigger pulse: 10µs high on PA0
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    DWT_Delay_us(10);  // microsecond delay via DWT cycle counter
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    // Wait for echo high
    uint32_t start = DWT->CYCCNT;
    while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
        if ((DWT->CYCCNT - start) > 168000) return -1; // timeout
    }

    uint32_t t1 = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) {
        if ((DWT->CYCCNT - t1) > 1680000) return -1;  // timeout
    }
    uint32_t t2 = DWT->CYCCNT;

    // Convert cycles to distance
    // Distance(m) = (cycles / 168MHz) * 343 / 2
    float duration_us = (t2 - t1) / 168.0f;
    return (duration_us * 0.0343f) / 2.0f;
}

// ── MAIN LOOP ─────────────────────────────────────────
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();

    // Init DWT for µs timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Init NRF24 as transmitter
    NRF24_Init();
    NRF24_TxMode(0xB3B4B5B6F1, 76); // address, channel

    // Init MPU6050
    MPU6050_Init(&hi2c1);

    VehicleData_t data;

    while (1) {
        // 1. Read distance
        data.distance_m = HC_SR04_GetDistance();

        // 2. Read acceleration magnitude from MPU6050
        MPU6050_t mpu;
        MPU6050_Read_Accel(&hi2c1, &mpu);
        data.accel_g = sqrtf(mpu.Ax*mpu.Ax +
                             mpu.Ay*mpu.Ay +
                             mpu.Az*mpu.Az);

        // 3. Read brake button (active low)
        data.brake_alert = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

        // 4. Transmit over NRF24
        NRF24_Transmit((uint8_t*)&data, sizeof(data));

        HAL_Delay(200); // send every 200ms
    }
}
```
