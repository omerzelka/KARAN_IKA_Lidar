/* ==========================================================================
 * uart_relay.c  —  STM32F4 UART köprüsü (Jetson USART1 -> PC USART2)
 *
 * Tasarım:
 *  - USART1 RX kesme (interrupt) ile tek tek bayt alınır -> halka tampon.
 *  - Ana döngü Relay_Task() halka tamponu USART2'ye boşaltır.
 *  - İki UART da 57600 8N1 olduğundan hız eşittir; kopyalama triviyaldir,
 *    taşma olmaz. (Jetson->STM32 daha hızlı olsaydı akış kontrolü gerekirdi.)
 *
 * Not: HAL_UART_Transmit'i ISR içinde ÇAĞIRMIYORUZ (bloklar). TX ana
 *      döngüde polling ile yapılır; RX ise kesmede sadece tampona yazar.
 * ========================================================================== */
#include "uart_relay.h"

#define RB_SIZE 1024u                 /* 2'nin katı olmalı (maske ile hızlı mod) */
#define RB_MASK (RB_SIZE - 1u)

static volatile uint8_t  s_buf[RB_SIZE];
static volatile uint16_t s_head = 0;  /* ISR yazar */
static volatile uint16_t s_tail = 0;  /* ana döngü okur */

static UART_HandleTypeDef *s_from = 0;  /* Jetson (RX) */
static UART_HandleTypeDef *s_to   = 0;  /* PC (TX)     */
static uint8_t s_rx_byte;               /* HAL_UART_Receive_IT hedefi */

void Relay_Init(UART_HandleTypeDef *from_jetson, UART_HandleTypeDef *to_pc)
{
    s_from = from_jetson;
    s_to   = to_pc;
    s_head = s_tail = 0;
    /* İlk baytı beklemeye başla; her bayt sonrası callback yeniden başlatır */
    HAL_UART_Receive_IT(s_from, &s_rx_byte, 1);
}

void Relay_OnRxByte(UART_HandleTypeDef *huart)
{
    if (huart != s_from) {
        return;
    }
    uint16_t next = (uint16_t)((s_head + 1u) & RB_MASK);
    if (next != s_tail) {          /* tampon dolu değilse yaz (dolyusa baytı düşür) */
        s_buf[s_head] = s_rx_byte;
        s_head = next;
    }
    /* Bir sonraki baytı beklemeye devam et */
    HAL_UART_Receive_IT(s_from, &s_rx_byte, 1);
}

void Relay_Task(void)
{
    /* Tamponda biriken tüm baytları PC UART'ına aktar */
    while (s_tail != s_head) {
        uint8_t b = s_buf[s_tail];
        /* 57600'de tek bayt ~174us; kısa timeout yeter */
        if (HAL_UART_Transmit(s_to, &b, 1, 5) == HAL_OK) {
            s_tail = (uint16_t)((s_tail + 1u) & RB_MASK);
        } else {
            break;  /* TX meşgul -> sonraki Relay_Task çağrısında devam et */
        }
    }
}

/* ==========================================================================
 * CubeMX üretimi Core/Src/stm32f4xx_it.c ya da main.c içindeki
 * HAL_UART_RxCpltCallback'e şunu ekle (zaten varsa içine Relay_OnRxByte koy):
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       Relay_OnRxByte(huart);
 *   }
 * ========================================================================== */
