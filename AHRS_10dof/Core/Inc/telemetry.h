#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "stm32f4xx_hal.h"   /* Kullandığın seriye göre değiştir (f1xx, f4xx vb.) */
#include <stdint.h>

/*
 * Telemetry modülü
 * -----------------
 * Roll, pitch, yaw (derece) ve altitude (metre) değerlerini belirli bir
 * periyotla UART üzerinden PC'ye gönderir.
 *
 * Non-blocking çalışır: Telemetry_Update() her main loop turunda çağrılır,
 * ama gönderim sadece periyot dolduğunda ve önceki transfer bittiğinde
 * gerçekleşir. HAL_Delay yok, main döngüsünü kilitlemez.
 *
 * Format (PC tarafında parse edilecek):
 *   "roll,pitch,yaw,altitude\r\n"
 *   örnek: "12.34,-5.67,90.12,153.20\r\n"
 */

void Telemetry_Init(UART_HandleTypeDef *huart, uint32_t period_ms);
void Telemetry_Update(float roll_deg, float pitch_deg, float yaw_deg, float altitude_m);

#endif /* TELEMETRY_H */
