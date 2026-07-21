#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "ydlidar_tmini_driver/serial_transport.hpp"
#include "ydlidar_tmini_driver/ydlidar_protocol.hpp"

namespace ydlidar_tmini_driver
{

// ============================================================================
// YDLIDAR T-mini Plus ROS 2 sürücü düğümü.
//
// Akış:
//   1. Ayrı okuyucu thread'i seri portu açar (230400 baud), STOP gönderir,
//      GET_HEALTH ile cihazı doğrular ve SCAN (0xA5 0x60) komutunu gönderir.
//   2. Cevap descriptor'ı (0xA5 0x5A ... 0x81) doğrulanır; ardından çok noktalı
//      VERİ PAKETLERİ (0x55 0xAA başlıklı) akmaya başlar.
//   3. Her paket başlığı bulunur, LSN örnek FSA/LSA arası interpolasyonla
//      açı+mesafeye çözümlenir. CT&0x01 (yeni turun ilk paketi) görülünce
//      biriken noktalar LaserScan'e dönüştürülüp /scan'e basılır.
//
// Tüm seri G/Ç okuyucu thread'inde yapılır; ROS executor'ı asla bloklanmaz.
// Publisher QoS'u SensorDataQoS'tur (Best Effort + Volatile).
// ============================================================================
class YdlidarDriverNode : public rclcpp::Node
{
public:
  explicit YdlidarDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~YdlidarDriverNode() override;

  YdlidarDriverNode(const YdlidarDriverNode &) = delete;
  YdlidarDriverNode & operator=(const YdlidarDriverNode &) = delete;

private:
  void readerLoop();
  bool connectAndStartScan();
  bool checkHealth();
  bool sendCommand(uint8_t cmd);
  bool waitResponseDescriptor(uint8_t expected_type, uint32_t & payload_len, int timeout_ms);
  bool readExact(uint8_t * buffer, size_t len, int timeout_ms);
  int  fillRxBuffer(int timeout_ms);

  // Tampondaki tüm tam paketleri çözümler; yeni tur başında yayınlar
  void parseBufferedPackets();

  // Biriken tam turu LaserScan mesajına dönüştürüp yayınlar
  void publishScan();

  // (Opsiyonel) DTR üzerinden motor kontrolü — T-mini Plus'ta genelde gereksiz
  void startMotor();
  void stopMotor();

  void shutdownLidar();

  // Bir turda beklenen minimum nokta sayısı (ilk eksik turu yayınlamamak için)
  static constexpr size_t kMinPointsPerScan = 60;
  // Ring-start hiç gelmezse belleğin sınırsız büyümesini engelleyen üst sınır
  static constexpr size_t kMaxPointsPerScan = 8192;

  // --- Parametreler ---
  std::string port_;
  uint32_t    baud_rate_{230400};
  std::string frame_id_;
  double      range_min_{0.02};
  double      range_max_{12.0};
  size_t      num_bins_{360};
  int         sample_bytes_{2};          // 2 = mesafe-only, 3 = yoğunluklu varyant
  double      distance_scale_mm_{0.25};  // ham -> mm  (standart = 1/4)
  bool        angle_correction_{false};  // üçgenleme düzeltmesi (ToF'ta kapalı)
  bool        verify_checksum_{false};   // CS doğrulaması (donanımda test edip aç)
  bool        invert_angle_{true};       // CW(lidar) -> CCW(REP-103) aynalama
  bool        use_dtr_motor_{false};     // DTR ile motor kontrolü (nadiren gerekli)
  bool        health_check_{true};       // GET_HEALTH doğrulaması

  // --- Donanım ve ROS arayüzleri ---
  SerialTransport serial_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread       reader_thread_;
  std::atomic<bool> running_{false};

  // --- Durum (yalnızca okuyucu thread'i erişir) ---
  std::vector<uint8_t>             rx_buffer_;    // ham bayt akışı tamponu
  std::vector<protocol::ScanPoint> scan_buffer_;  // bir tam tura ait noktalar
  rclcpp::Time scan_start_stamp_;                 // turun ilk paketinin zamanı
  uint64_t     bad_packet_count_{0};              // atılan bozuk paket sayısı
};

}  // namespace ydlidar_tmini_driver
