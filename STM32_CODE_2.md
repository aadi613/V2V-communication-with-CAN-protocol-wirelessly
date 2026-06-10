**The code for second STM32 board for receiving the data from other stm32 to this board**

```
// main.c - Board 2 RX side
// Uses: NRF24L01 lib, SSD1306 OLED lib

#include "main.h"
#include "nrf24l01.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    float distance_m;
    float accel_g;
    uint8_t brake_alert;
} VehicleData_t;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();

    // Init NRF24 as receiver
    NRF24_Init();
    NRF24_RxMode(0xB3B4B5B6F1, 76); // same address + channel as TX

    // Init OLED
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 20);
    ssd1306_WriteString("Waiting...", Font_7x10, White);
    ssd1306_UpdateScreen();

    VehicleData_t rxData;
    char buf[32];

    while (1) {
        if (NRF24_IsDataAvailable(1)) { // pipe 1
            NRF24_Receive((uint8_t*)&rxData, sizeof(rxData));

            // Update OLED
            ssd1306_Fill(Black);

            // Line 1: Distance
            snprintf(buf, sizeof(buf), "Dist: %.1fm", rxData.distance_m);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString(buf, Font_7x10, White);

            // Line 2: Acceleration
            snprintf(buf, sizeof(buf), "Accel: %.2fg", rxData.accel_g);
            ssd1306_SetCursor(0, 16);
            ssd1306_WriteString(buf, Font_7x10, White);

            // Line 3: Brake alert
            ssd1306_SetCursor(0, 32);
            if (rxData.brake_alert) {
                ssd1306_WriteString("!! BRAKE ALERT !!", Font_7x10, White);
            } else {
                ssd1306_WriteString("Status: OK", Font_7x10, White);
            }

            ssd1306_UpdateScreen();
        }
        HAL_Delay(10);
    }
}
```
