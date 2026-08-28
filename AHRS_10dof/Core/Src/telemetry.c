#include "telemetry.h"
#include <stdio.h>

static UART_HandleTypeDef *s_huart   = NULL;
static uint32_t            s_period_ms = 50;
static uint32_t            s_last_tick = 0;

/* ÖNEMLİ: Bu buffer static olmak zorunda.
 * HAL_UART_Transmit_IT asenkron çalışır; fonksiyon geri döndükten sonra
 * bile arka planda gönderim devam eder. Eğer buf stack üzerinde (local)
 * olsaydı, fonksiyon return ettiği anda geçersiz hale gelirdi ve
 * gönderilen veri bozuk/rastgele çıkardı. Bu, IT/DMA tabanlı UART
 * kullanırken en sık yapılan hatalardan biri. */
static char s_buf[64];

void Telemetry_Init(UART_HandleTypeDef *huart, uint32_t period_ms)
{
    s_huart     = huart;
    s_period_ms = period_ms;
    s_last_tick = HAL_GetTick();
}

void Telemetry_Update(float roll_deg, float pitch_deg, float yaw_deg, float altitude_m)
{
    if (s_huart == NULL) {
        return; /* Init çağrılmadıysa hiçbir şey yapma */
    }

    uint32_t now = HAL_GetTick();
    if ((now - s_last_tick) < s_period_ms) {
        return; /* Periyot dolmadı, non-blocking çıkış */
    }

    /* Önceki transfer hâlâ devam ediyorsa bu turu atla.
     * Bir sonraki Update çağrısında tekrar denenecek, ama s_last_tick
     * güncellendiği için "kaçırılan" tur periyodu bozmaz. */
    if (s_huart->gState != HAL_UART_STATE_READY) {
        return;
    }

    s_last_tick = now;

		int len = snprintf(s_buf, sizeof(s_buf), "%.2f,%.2f,%.2f,%.2f\r\n",
							roll_deg, pitch_deg, yaw_deg, altitude_m);

    if (len > 0 && len < (int)sizeof(s_buf)) {
        HAL_UART_Transmit_IT(s_huart, (uint8_t *)s_buf, (uint16_t)len);
    }
}
