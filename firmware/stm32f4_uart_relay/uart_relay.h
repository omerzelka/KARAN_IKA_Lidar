/* ==========================================================================
 * uart_relay.h  —  STM32F4 (F401/F407) UART köprüsü
 *
 * Jetson'dan (USART1 @ 57600) gelen kompakt lidar string'ini PC'ye
 * (USART2 @ 57600) olduğu gibi aktarır. Dumb relay: içeriği yorumlamaz,
 * sadece bayt akışını geçirir. İçerik $L/$P NMEA benzeri satırlardır.
 *
 * KULLANIM (CubeMX üretimi main.c içinde):
 *   USER CODE BEGIN 2:      Relay_Init(&huart1, &huart2);
 *   USER CODE BEGIN WHILE:  Relay_Task();
 *   HAL_UART_RxCpltCallback: Relay_OnRxByte(huart);   (bkz. uart_relay.c)
 * ========================================================================== */
#ifndef UART_RELAY_H
#define UART_RELAY_H

#include "main.h"   /* HAL + huartX handle tanımları */

/* from_jetson : lidar string'inin GELDİĞİ UART (RX), örn &huart1
 * to_pc       : string'in GİDECEĞİ UART (TX),     örn &huart2  */
void Relay_Init(UART_HandleTypeDef *from_jetson, UART_HandleTypeDef *to_pc);

/* Ana döngüde sürekli çağrılır: RX halka tamponunu PC UART'ına boşaltır. */
void Relay_Task(void);

/* HAL_UART_RxCpltCallback içinden çağrılır (gelen baytı tampona koyar). */
void Relay_OnRxByte(UART_HandleTypeDef *huart);

#endif /* UART_RELAY_H */
