#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace rplidar_a2m12_driver
{

// ============================================================================
// Alt seviye POSIX seri port katmanı (Linux'a özgü termios2 tabanlı).
//
// Tasarım kararları:
//  - 256000 baud, POSIX'in standart Bxxxxx sabitlerinde YOKTUR. Bu yüzden
//    Linux'un termios2 + BOTHER mekanizmasıyla isteğe bağlı (arbitrary)
//    baud rate ayarlanır. Aynı mekanizma 115200 gibi standart hızlarda da
//    sorunsuz çalışır (Jetson Orin Nano / Ubuntu 22.04 kernel'i destekler).
//  - read() asla bloklamaz (O_NONBLOCK + VMIN=0/VTIME=0). Bekleme işi
//    poll() ile kernel'e bırakılır: veri gelene kadar thread uyur,
//    CPU kullanımı ~%0 olur. (while(1) busy-wait YOKTUR.)
//  - DTR hattı ioctl(TIOCMBIS/TIOCMBIC) ile sürülür; Slamtec USB adaptöründe
//    bu hat motor sürücüsünün (MOTOCTL) PWM enable girişine bağlıdır.
// ============================================================================
class SerialTransport
{
public:
  SerialTransport() = default;
  ~SerialTransport();

  // Kopyalanamaz: dosya tanımlayıcısının (fd) tek sahibi bu sınıf olmalıdır
  SerialTransport(const SerialTransport &) = delete;
  SerialTransport & operator=(const SerialTransport &) = delete;

  // Portu açar, kilitler (flock) ve 8N1 raw modda verilen baud'a ayarlar.
  // Hata durumunda false döner ve error_msg'e açıklama yazar.
  bool openPort(const std::string & device, uint32_t baud_rate, std::string & error_msg);

  void closePort();

  bool isOpen() const { return fd_ >= 0; }

  // En fazla timeout_ms milisaniye veri bekler (poll ile kernel'de uyuyarak).
  // Dönüş: okunan bayt sayısı (>0), 0 = zaman aşımı (veri yok),
  //        -1 = kalıcı hata (örn. USB kablosu çekildi).
  int readBytes(uint8_t * buffer, size_t max_len, int timeout_ms);

  // Tüm baytlar yazılana kadar dener (kısmi write'ları tamamlar).
  bool writeBytes(const uint8_t * data, size_t len);

  // DTR modem hattını sürer. Slamtec USB adaptöründe:
  //   DTR LOW  (level=false) -> motor DÖNER
  //   DTR HIGH (level=true)  -> motor DURUR
  bool setDtr(bool level);

  // Kernel'in giriş (RX) tamponunu boşaltır.
  void flushInput();

private:
  // termios2 + BOTHER ile 8N1 raw mod konfigürasyonu. İmplementasyon .cpp'de:
  // <asm/termbits.h> ile <termios.h> çakıştığı için başlıkta tutulamaz.
  bool configurePort(uint32_t baud_rate, std::string & error_msg);

  int fd_{-1};
};

}  // namespace rplidar_a2m12_driver
