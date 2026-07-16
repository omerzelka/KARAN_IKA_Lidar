#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rplidar_a2m12_driver/rplidar_protocol.hpp"
#include "rplidar_a2m12_driver/serial_transport.hpp"

namespace rplidar_a2m12_driver
{

// ============================================================================
// RPLIDAR A2M12 ROS 2 sürücü düğümü.
//
// Akış:
//   1. Ayrı bir okuyucu thread'i seri portu açar, motoru DTR ile döndürür,
//      GET_HEALTH ile cihazı doğrular ve SCAN (0xA5 0x20) komutunu gönderir.
//   2. Durum makinesi: önce 0xA5 0x5A cevap descriptor'ı aranır (senkronizasyon),
//      sonra 5 baytlık ölçüm düğümleri ayrıştırılır. Bozuk paketlerde akış
//      1 bayt kaydırılarak yeniden senkronize edilir (RCLCPP_WARN basılır).
//   3. Start flag'li düğüm geldiğinde bir 360 derecelik tur tamamlanmıştır:
//      biriken noktalar LaserScan mesajına dönüştürülüp /scan'e basılır.
//
// Tüm seri port işlemleri okuyucu thread'inde yapılır; ROS executor'ı asla
// bloklanmaz. Publisher QoS'u SensorDataQoS'tur (Best Effort + Volatile).
// ============================================================================
class RplidarDriverNode : public rclcpp::Node
{
public:
  explicit RplidarDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RplidarDriverNode() override;

  RplidarDriverNode(const RplidarDriverNode &) = delete;
  RplidarDriverNode & operator=(const RplidarDriverNode &) = delete;

private:
  // --- Okuyucu thread'inin ana döngüsü: bağlan -> oku -> ayrıştır -> yayınla ---
  void readerLoop();

  // Port açma + motor başlatma + sağlık kontrolü + SCAN komutu + descriptor doğrulama
  bool connectAndStartScan();

  // GET_HEALTH sorgusu; "Protection Stop" durumunda RESET gönderir
  bool checkHealth();

  // Komut paketi gönderir (payload'lı komutlarda checksum otomatik eklenir)
  bool sendCommand(uint8_t cmd, const uint8_t * payload = nullptr, uint8_t payload_len = 0);

  // Bayt akışında 0xA5 0x5A descriptor'ını arar ve tipini doğrular
  bool waitResponseDescriptor(uint8_t expected_type, uint32_t & payload_len, int timeout_ms);

  // Tam olarak 'len' bayt okunana kadar bekler (zaman aşımı dahilinde)
  bool readExact(uint8_t * buffer, size_t len, int timeout_ms);

  // Seri porttan okuyup rx_buffer_'a ekler; okunan bayt sayısını döner (-1 = hata)
  int fillRxBuffer(int timeout_ms);

  // Tek bir ölçüm düğümünü işler; start flag'de turu yayınlar
  void handleNode(const protocol::ScanNode & node);

  // Biriken tam turu LaserScan mesajına dönüştürüp yayınlar
  void publishScan();

  // DTR üzerinden motor kontrolü (USB adaptör kartındaki PWM'i tetikler)
  void startMotor();
  void stopMotor();

  // STOP komutu + motor durdurma + port kapatma (okuyucu thread çıkışında)
  void shutdownLidar();

  // Bir turda beklenen minimum nokta sayısı (ilk eksik turu yayınlamamak için)
  static constexpr size_t kMinNodesPerScan = 100;
  // Start flag hiç gelmezse belleğin sınırsız büyümesini engelleyen üst sınır
  static constexpr size_t kMaxNodesPerScan = 8192;

  // --- Parametreler ---
  std::string port_;
  uint32_t    baud_rate_{256000};
  std::string frame_id_;
  double      range_min_{0.15};
  double      range_max_{12.0};
  size_t      num_bins_{720};

  // --- Donanım ve ROS arayüzleri ---
  SerialTransport serial_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread       reader_thread_;
  std::atomic<bool> running_{false};

  // --- Durum (yalnızca okuyucu thread'i erişir) ---
  std::vector<uint8_t>            rx_buffer_;    // ham bayt akışı tamponu
  std::vector<protocol::ScanNode> scan_buffer_;  // bir tam tura ait noktalar
  rclcpp::Time scan_start_stamp_;                // turun ilk ölçümünün zamanı
  uint64_t     corrupt_byte_count_{0};           // toplam atılan bozuk bayt sayısı
};

}  // namespace rplidar_a2m12_driver
