#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ydlidar_tmini_driver
{

// ============================================================================
// Alt seviye POSIX seri port katmanı (Linux'a özgü termios2 tabanlı).
//
// Model-bağımsızdır: RPLIDAR sürücüsündeki katmanla aynıdır, yalnızca baud
// (T-mini Plus = 230400) değişir. 230400 standart bir POSIX baud sabitidir
// (B230400) ama termios2 + BOTHER yolu her değeri (200000, 460800, ...)
// desteklediği için kod genel tutulmuştur.
//
// Tasarım kararları:
//  - read() asla bloklamaz (O_NONBLOCK + VMIN=0/VTIME=0). Bekleme işi poll()
//    ile kernel'e bırakılır: veri gelene kadar thread uyur, CPU ~%0.
//  - DTR hattı ioctl(TIOCMBIS/TIOCMBIC) ile sürülür. YDLIDAR T-mini Plus'ta
//    motor SCAN komutuyla başlar; DTR motor kontrolü GEREKMEZ. Yine de bazı
//    USB adaptör kartları için hat erişilebilir bırakıldı (bkz. driver_node
//    'use_dtr_motor' parametresi).
// ============================================================================
class SerialTransport
{
public:
  SerialTransport() = default;
  ~SerialTransport();

  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  // Portu açar, kilitler (flock) ve 8N1 raw modda verilen baud'a ayarlar.
  bool openPort(const std::string & device, uint32_t baud_rate, std::string & error_msg);

  void closePort();

  bool isOpen() const { return fd_ >= 0; }

  // En fazla timeout_ms ms veri bekler (poll ile kernel'de uyuyarak).
  // Dönüş: >0 okunan bayt, 0 zaman aşımı, -1 kalıcı hata (kablo çekildi vb.).
  int readBytes(uint8_t * buffer, size_t max_len, int timeout_ms);

  // Tüm baytlar yazılana kadar dener (kısmi write'ları tamamlar).
  bool writeBytes(const uint8_t * data, size_t len);

  // DTR modem hattını sürer (level=true -> DTR HIGH, false -> DTR LOW).
  bool setDtr(bool level);

  // Kernel'in giriş (RX) tamponunu boşaltır.
  void flushInput();

private:
  bool configurePort(uint32_t baud_rate, std::string & error_msg);

  int fd_{-1};
};

}  // namespace ydlidar_tmini_driver
