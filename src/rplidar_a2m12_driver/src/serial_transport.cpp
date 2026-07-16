#include "rplidar_a2m12_driver/serial_transport.hpp"

// ============================================================================
// DİKKAT: Bu çeviri biriminde <termios.h> KULLANILMAZ!
// <asm/termbits.h> (struct termios2, BOTHER) ile <termios.h> aynı dosyada
// derlenemez (struct termios yeniden tanım çatışması). 256000 baud standart
// POSIX sabitlerinde olmadığından tüm konfigürasyon ioctl(TCGETS2/TCSETS2)
// üzerinden yapılır.
// ============================================================================
#include <asm/termbits.h>  // struct termios2, BOTHER, CBAUD, CS8, TCIFLUSH...
#include <fcntl.h>         // open, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <poll.h>          // poll, POLLIN, POLLOUT
#include <sys/file.h>      // flock
#include <sys/ioctl.h>     // ioctl, TCGETS2, TCSETS2, TIOCMBIS/TIOCMBIC, TIOCM_DTR
#include <unistd.h>        // read, write, close

#include <cerrno>
#include <cstring>

namespace rplidar_a2m12_driver
{

SerialTransport::~SerialTransport()
{
  closePort();
}

bool SerialTransport::openPort(const std::string & device, uint32_t baud_rate,
                               std::string & error_msg)
{
  closePort();

  // O_NOCTTY  : port, prosesin kontrol terminali olmasın (sinyal karışmasın)
  // O_NONBLOCK: read()/write() asla bloklamasın; bekleme poll() ile yapılacak
  // O_CLOEXEC : fork+exec durumunda fd çocuk prosese sızmasın
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error_msg = std::string("open() hatası: ") + std::strerror(errno);
    return false;
  }

  // Aynı portu iki prosesin aynı anda kullanmasını engelle (üretim güvenliği)
  if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
    error_msg = "Port başka bir proses tarafından kullanılıyor (flock kilidi alınamadı)";
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (!configurePort(baud_rate, error_msg)) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  flushInput();
  return true;
}

bool SerialTransport::configurePort(uint32_t baud_rate, std::string & error_msg)
{
  struct termios2 tio;
  std::memset(&tio, 0, sizeof(tio));

  if (::ioctl(fd_, TCGETS2, &tio) < 0) {
    error_msg = std::string("ioctl(TCGETS2) hatası: ") + std::strerror(errno);
    return false;
  }

  // --- Çerçeve: 8 veri biti, parite yok, 1 stop biti (8N1), akış kontrolü yok ---
  tio.c_cflag &= ~static_cast<tcflag_t>(CBAUD | CSIZE | PARENB | CSTOPB | CRTSCTS);
  tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
  //              ^BOTHER: standart Bxxxxx sabitleri yerine c_ispeed/c_ospeed
  //              alanlarındaki HAM sayısal değeri kullan (256000 böyle ayarlanır)

  // --- Giriş: tamamen ham (raw) mod; hiçbir bayt dönüştürülmesin/yutulmasın ---
  tio.c_iflag &= ~static_cast<tcflag_t>(
    IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

  // --- Çıkış: post-processing kapalı (binary komut paketleri bozulmasın) ---
  tio.c_oflag &= ~static_cast<tcflag_t>(OPOST | ONLCR);

  // --- Yerel mod: kanonik mod, echo ve sinyal üretimi kapalı ---
  tio.c_lflag &= ~static_cast<tcflag_t>(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

  // İsteğe bağlı baud rate (256000 dahil her değer)
  tio.c_ispeed = baud_rate;
  tio.c_ospeed = baud_rate;

  // VMIN=0, VTIME=0: read() elde ne varsa hemen döner, hiç bloklamaz.
  // Veri bekleme işi poll()'a bırakılır -> CPU boşta uyur.
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (::ioctl(fd_, TCSETS2, &tio) < 0) {
    error_msg = std::string("ioctl(TCSETS2) hatası: ") + std::strerror(errno);
    return false;
  }
  return true;
}

int SerialTransport::readBytes(uint8_t * buffer, size_t max_len, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }

  struct pollfd pfd;
  pfd.fd      = fd_;
  pfd.events  = POLLIN;
  pfd.revents = 0;

  // poll(): veri gelene veya süre dolana kadar thread KERNEL'de uyur.
  // Busy-wait yoktur; Jetson'un CPU çekirdekleri YOLO gibi işlere kalır.
  const int poll_result = ::poll(&pfd, 1, timeout_ms);
  if (poll_result < 0) {
    // Sinyal kesmesi (EINTR) kalıcı hata değildir; zaman aşımı gibi davran
    return (errno == EINTR) ? 0 : -1;
  }
  if (poll_result == 0) {
    return 0;  // zaman aşımı: bu pencerede veri gelmedi
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;  // USB kablosu çekilmiş veya sürücü hatası -> yeniden bağlantı gerekir
  }

  const ssize_t n = ::read(fd_, buffer, max_len);
  if (n < 0) {
    return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
  }
  return static_cast<int>(n);
}

bool SerialTransport::writeBytes(const uint8_t * data, size_t len)
{
  if (fd_ < 0) {
    return false;
  }

  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        // Çıkış tamponu dolu: yer açılana kadar (en fazla 100 ms) uyu
        struct pollfd pfd;
        pfd.fd      = fd_;
        pfd.events  = POLLOUT;
        pfd.revents = 0;
        if (::poll(&pfd, 1, 100) <= 0) {
          return false;
        }
        continue;
      }
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool SerialTransport::setDtr(bool level)
{
  if (fd_ < 0) {
    return false;
  }
  int modem_flag = TIOCM_DTR;
  // TIOCMBIS: biti set et (DTR HIGH -> motor durur)
  // TIOCMBIC: biti temizle (DTR LOW -> USB kartındaki PWM devreye girer, motor döner)
  return ::ioctl(fd_, level ? TIOCMBIS : TIOCMBIC, &modem_flag) == 0;
}

void SerialTransport::flushInput()
{
  if (fd_ >= 0) {
    ::ioctl(fd_, TCFLSH, TCIFLUSH);
  }
}

void SerialTransport::closePort()
{
  if (fd_ >= 0) {
    ::close(fd_);  // close() aynı zamanda flock kilidini de serbest bırakır
    fd_ = -1;
  }
}

}  // namespace rplidar_a2m12_driver
