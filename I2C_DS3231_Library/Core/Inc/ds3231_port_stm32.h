#ifndef DS3231_PORT_STM32_H
#define DS3231_PORT_STM32_H

#include "ds3231.h"
#include "stm32f4xx_hal.h"

/*
 * STM32 HAL'e ozel "port" katmani.
 *
 * Genel ds3231.h/.c kutuphanesi HAL bilmez. HAL ile temas eden tek
 * yer burasidir. Baska bir platforma gecerken (Arduino, ESP-IDF, ...)
 * sadece bu iki dosyayi (ds3231_port_stm32.h/.c) o platforma gore
 * yeniden yazman yeterli, ds3231.h/.c'ye dokunmana gerek yok.
 */

int8_t DS3231_Port_STM32_Init(DS3231_Handle_t *handle, I2C_HandleTypeDef *hi2c);

#endif /* DS3231_PORT_STM32_H */
