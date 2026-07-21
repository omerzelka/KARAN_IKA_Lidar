# STM32F4 UART Relay — Kurulum

Jetson'dan gelen kompakt lidar **binary çerçevelerini** PC'ye aktaran köprü
firmware'i. `Lidar → Jetson → USART1(STM32) → USART2(STM32) → PC` zincirindeki
STM32 halkası. Relay **byte-transparan**: baytları yorumlamadan aktarır, bu
yüzden binary veri (`0xAA 0x55` sync, `\r`, `\n`, `$` dahil) sorunsuz geçer.

## Bağlantı (F401/F407 tipik pinler)

| Hat | STM32 pini | Karşı taraf |
|-----|-----------|-------------|
| USART1_RX | PA10 | Jetson UART **TX** (örn. `/dev/ttyTHS1` TX) |
| USART1_TX | PA9  | (kullanılmıyor, tek yön) |
| USART2_TX | PA2  | PC UART **RX** (USB-TTL adaptörü RX) |
| USART2_RX | PA3  | (kullanılmıyor) |
| GND | GND | **Ortak GND şart** (Jetson, STM32, PC hepsi) |

⚠️ **Seviye:** Jetson ve STM32 3.3V mantık — doğrudan bağlanır. PC tarafında
USB-TTL adaptörü mutlaka **3.3V** olsun (5V TX, STM32 RX'ini yakabilir).

## CubeMX ayarları
1. **USART1**: Asynchronous, Baud **57600**, 8N1, parity none.
   - NVIC → **USART1 global interrupt = ENABLED** (RX kesmesi için şart).
2. **USART2**: Asynchronous, Baud **57600**, 8N1, parity none. (Kesme gerekmez.)
3. Clock: HSE/PLL ile 57600 hata payı düşük olacak şekilde (F407 84/168 MHz sorunsuz).
4. Kod üret (Generate Code).

## Kodu ekleme
1. `uart_relay.c` ve `uart_relay.h`'yi projeye kopyala (Core/Src, Core/Inc).
2. `main.c`:
   ```c
   /* USER CODE BEGIN Includes */
   #include "uart_relay.h"
   /* USER CODE END Includes */

   /* USER CODE BEGIN 2 */
   Relay_Init(&huart1, &huart2);   /* huart1=Jetson, huart2=PC */
   /* USER CODE END 2 */

   /* USER CODE BEGIN WHILE */
   while (1) {
       Relay_Task();
       /* USER CODE END WHILE */
       /* USER CODE BEGIN 3 */
   }
   ```
3. `HAL_UART_RxCpltCallback` (main.c USER CODE 4 ya da it.c):
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
       Relay_OnRxByte(huart);
   }
   ```
4. Derle, flash'la.

## Doğrulama
- Jetson'da `scan_serial_bridge.py` çalışırken, STM32 üzerinden PC'de
  `scan_serial_receiver.py` `tur seq=... nokta=...` satırlarını basmalı
  (CRC'yi doğrulayıp binary çerçeveleri çözer).
- Görmüyorsan: GND ortak mı, TX/RX çaprazlanmış mı (bir tarafın TX'i diğerinin
  RX'ine), baud iki tarafta da 57600 mü — sırayla kontrol et.

## Genişletme
STM32 başka telemetri de gönderiyorsa (motor, IMU...), onları **aynı binary
çerçeveyle farklı `type` baytı** kullanarak (örn. `'M'`, `'I'`) aynı USART2'den
yolla. PC alıcısının `parse_buffer`'ı bilmediği `type`'ları CRC'si tuttuğu sürece
güvenle atlar (`len` ile boyunu bilir), `L`/`P` dışını yok sayar. Böylece tek
57600 hattı paylaşılır.
