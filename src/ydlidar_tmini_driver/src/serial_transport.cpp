#include "ydlidar_tmini_driver/serial_transport.hpp"

// ============================================================================
// DİKKAT: Bu çeviri biriminde <termios.h> KULLANILMAZ!
// <asm/termbits.h> (struct termios2, BOTHER) ile <termios.h> aynı dosyada
// derlenemez (struct termios yeniden tanım çatışması). Konfigürasyon tümüyle
// ioctl(TCGETS2/TCSETS2) üzerinden yapılır; bu isteğe bağlı baud rate'i de
// (BOTHER + c_ispeed/c_ospeed) mümkün kılar.
// ============================================================================
#include <asm/termbits.h>  // struct termios2, BOTHER, CBAUD, CS8, TCIFLUSH...
#include <fcntl.h>         // open, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <poll.h>          // poll, POLLIN, POLLOUT
#include <sys/file.h>      // flock
#include <sys/ioctl.h>     // ioctl, TCGETS2, TCSETS2, TIOCMBIS/TIOCMBIC, TIOCM_DTR
#include <unistd.h>        // read, write, close

#include <cerrno>
#include <cstring>

namespace ydlidar_tmini_driver
{

SerialTransport::~SerialTransport()
{
  closePort();
}

bool SerialTransport::openPort(const std::string & device, uint32_t baud_rate,
                               std::string & error_msg)
{
  closePort();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error_msg = std::string("open() hatası: ") + std::strerror(errno);
    return false;
  }

  // Aynı portu iki prosesin aynı anda kullanmasını engelle
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

  // --- Çerçeve: 8N1, akış kontrolü yok ---
  tio.c_cflag &= ~static_cast<tcflag_t>(CBAUD | CSIZE | PARENB | CSTOPB | CRTSCTS);
  tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;

  // --- Giriş: tamamen ham (raw) mod ---
  tio.c_iflag &= ~static_cast<tcflag_t>(
    IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

  // --- Çıkış: post-processing kapalı ---
  tio.c_oflag &= ~static_cast<tcflag_t>(OPOST | ONLCR);

  // --- Yerel mod: kanonik/echo/sinyal kapalı ---
  tio.c_lflag &= ~static_cast<tcflag_t>(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);

  // İsteğe bağlı baud rate (230400 dahil her değer)
  tio.c_ispeed = baud_rate;
  tio.c_ospeed = baud_rate;

  // VMIN=0, VTIME=0: read() bloklamaz; bekleme poll()'a bırakılır.
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

  const int poll_result = ::poll(&pfd, 1, timeout_ms);
  if (poll_result < 0) {
    return (errno == EINTR) ? 0 : -1;
  }
  if (poll_result == 0) {
    return 0;  // zaman aşımı
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;  // kablo çekilmiş / sürücü hatası -> yeniden bağlantı gerekir
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
  // TIOCMBIS: biti set et (DTR HIGH), TIOCMBIC: biti temizle (DTR LOW)
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
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace ydlidar_tmini_driver
