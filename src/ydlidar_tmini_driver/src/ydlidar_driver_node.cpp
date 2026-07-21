#include "ydlidar_tmini_driver/ydlidar_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace ydlidar_tmini_driver
{

YdlidarDriverNode::YdlidarDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ydlidar_tmini_driver", options)
{
  // --- Parametreler (launch dosyasından veya CLI'dan geçersiz kılınabilir) ---
  port_              = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_rate_         = static_cast<uint32_t>(declare_parameter<int>("baud_rate", 230400));
  frame_id_          = declare_parameter<std::string>("frame_id", "laser_link");
  range_min_         = declare_parameter<double>("range_min", 0.02);
  range_max_         = declare_parameter<double>("range_max", 12.0);
  num_bins_          = static_cast<size_t>(declare_parameter<int>("num_bins", 360));
  sample_bytes_      = declare_parameter<int>("sample_bytes", 2);
  distance_scale_mm_ = declare_parameter<double>("distance_scale_mm", 0.25);
  angle_correction_  = declare_parameter<bool>("angle_correction", false);
  verify_checksum_   = declare_parameter<bool>("verify_checksum", false);
  invert_angle_      = declare_parameter<bool>("invert_angle", true);
  use_dtr_motor_     = declare_parameter<bool>("use_dtr_motor", false);
  health_check_      = declare_parameter<bool>("health_check", true);

  if (sample_bytes_ != 2 && sample_bytes_ != 3) {
    RCLCPP_WARN(get_logger(), "Geçersiz sample_bytes=%d; 2'ye zorlanıyor", sample_bytes_);
    sample_bytes_ = 2;
  }

  // Lidar gibi yüksek frekanslı akışlarda kayıp bir taramayı yeniden iletmek
  // anlamsızdır; Reliable QoS ağı tıkar. -> SensorDataQoS (Best Effort).
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  rx_buffer_.reserve(8192);
  scan_buffer_.reserve(4096);
  scan_start_stamp_ = now();

  RCLCPP_INFO(get_logger(),
              "YDLIDAR T-mini Plus sürücüsü başlatılıyor: port=%s baud=%u frame_id=%s "
              "sample_bytes=%d",
              port_.c_str(), baud_rate_, frame_id_.c_str(), sample_bytes_);

  running_.store(true);
  reader_thread_ = std::thread(&YdlidarDriverNode::readerLoop, this);
}

YdlidarDriverNode::~YdlidarDriverNode()
{
  running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

// ============================================================================
// OKUYUCU THREAD ANA DÖNGÜSÜ
//   [BAĞLI DEĞİL] -> connectAndStartScan() -> [PAKET AKIŞI]
//   Akışta her paket 0x55 0xAA başlığıyla gelir; başlığı tutmayan baytlar
//   tek tek atılarak yeniden senkronizasyon sağlanır.
// ============================================================================
void YdlidarDriverNode::readerLoop()
{
  bool connected = false;

  while (running_.load() && rclcpp::ok()) {
    if (!connected) {
      connected = connectAndStartScan();
      if (!connected) {
        std::this_thread::sleep_for(1s);
        continue;
      }
      scan_buffer_.clear();
      scan_start_stamp_ = now();
    }

    const int n = fillRxBuffer(200);
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "Seri port okuma hatası; yeniden bağlanılacak");
      serial_.closePort();
      connected = false;
      continue;
    }
    if (n == 0) {
      continue;  // zaman aşımı
    }

    parseBufferedPackets();
  }

  shutdownLidar();
}

// ============================================================================
// BAĞLANTI SIRASI: port aç -> STOP -> GET_HEALTH -> (ops. DTR motor)
//                  -> SCAN (0xA5 0x60) -> descriptor doğrula (tip 0x81)
// ============================================================================
bool YdlidarDriverNode::connectAndStartScan()
{
  std::string error_msg;
  if (!serial_.openPort(port_, baud_rate_, error_msg)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Seri port açılamadı (%s): %s", port_.c_str(), error_msg.c_str());
    return false;
  }
  rx_buffer_.clear();

  // Cihaz önceki oturumdan tarama modunda kalmış olabilir: önce durdur
  if (!sendCommand(protocol::CMD_STOP)) {
    serial_.closePort();
    return false;
  }
  std::this_thread::sleep_for(50ms);
  serial_.flushInput();
  rx_buffer_.clear();

  if (health_check_ && !checkHealth()) {
    serial_.closePort();
    return false;
  }

  // T-mini Plus'ta motor SCAN komutuyla döner; DTR yalnızca bazı adaptör
  // kartları için gerekebilir (parametre ile açılır).
  if (use_dtr_motor_) {
    startMotor();
    std::this_thread::sleep_for(1000ms);
  }

  // Start Scan: 0xA5 0x60
  if (!sendCommand(protocol::CMD_SCAN)) {
    RCLCPP_ERROR(get_logger(), "SCAN komutu gönderilemedi");
    if (use_dtr_motor_) { stopMotor(); }
    serial_.closePort();
    return false;
  }

  // Cevap descriptor'ı: 0xA5 0x5A ... data_type=0x81 (sürekli ölçüm akışı)
  uint32_t payload_len = 0;
  if (!waitResponseDescriptor(protocol::ANS_TYPE_MEASUREMENT, payload_len, 2000)) {
    RCLCPP_ERROR(get_logger(), "SCAN cevap descriptor'ı doğrulanamadı");
    if (use_dtr_motor_) { stopMotor(); }
    serial_.closePort();
    return false;
  }

  RCLCPP_INFO(get_logger(), "YDLIDAR taramaya başladı (%s @ %u baud)",
              port_.c_str(), baud_rate_);
  return true;
}

// ============================================================================
// GET_HEALTH: cevap descriptor'ı (tip 0x06) + 3 baytlık durum.
// status=2 (ERROR) durumunda RESET gönderip yeniden bağlanılır.
// ============================================================================
bool YdlidarDriverNode::checkHealth()
{
  if (!sendCommand(protocol::CMD_GET_HEALTH)) {
    return false;
  }

  uint32_t payload_len = 0;
  if (!waitResponseDescriptor(protocol::ANS_TYPE_DEVHEALTH, payload_len, 1000) ||
      payload_len < 3) {
    RCLCPP_ERROR(get_logger(),
                 "GET_HEALTH cevabı alınamadı (lidar bağlı mı? baud=230400 doğru mu?)");
    return false;
  }

  uint8_t health[3] = {0, 0, 0};
  if (!readExact(health, sizeof(health), 500)) {
    RCLCPP_ERROR(get_logger(), "GET_HEALTH payload'ı zaman aşımına uğradı");
    return false;
  }

  const uint8_t  status     = health[0];
  const uint16_t error_code = static_cast<uint16_t>(health[1] | (health[2] << 8));

  if (status == 2) {  // ERROR
    RCLCPP_ERROR(get_logger(),
                 "Lidar hata durumunda (kod: 0x%04X); RESET gönderiliyor", error_code);
    sendCommand(protocol::CMD_RESET);
    std::this_thread::sleep_for(2000ms);
    serial_.flushInput();
    rx_buffer_.clear();
    return false;
  }
  if (status == 1) {
    RCLCPP_WARN(get_logger(), "Lidar sağlık durumu: WARNING (kod: 0x%04X)", error_code);
  } else {
    RCLCPP_INFO(get_logger(), "Lidar sağlık durumu: OK");
  }
  return true;
}

bool YdlidarDriverNode::sendCommand(uint8_t cmd)
{
  // YDLIDAR istekleri payload'sız: yalnızca [0xA5][CMD]
  const uint8_t packet[2] = {protocol::SYNC_BYTE, cmd};
  return serial_.writeBytes(packet, sizeof(packet));
}

// ============================================================================
// DESCRIPTOR ARAMA: bayt akışında 0xA5 0x5A çiftini arar; bulunca 7 baytlık
// descriptor'ı çözümleyip tipini doğrular. (Slamtec ile aynı mantık.)
// ============================================================================
bool YdlidarDriverNode::waitResponseDescriptor(uint8_t expected_type,
                                               uint32_t & payload_len, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    bool   header_found = false;
    size_t header_pos   = 0;
    for (size_t i = 0; i + 1 < rx_buffer_.size(); ++i) {
      if (rx_buffer_[i] == protocol::SYNC_BYTE && rx_buffer_[i + 1] == protocol::SYNC_BYTE2) {
        header_found = true;
        header_pos   = i;
        break;
      }
    }

    if (header_found) {
      rx_buffer_.erase(rx_buffer_.begin(),
                       rx_buffer_.begin() + static_cast<std::ptrdiff_t>(header_pos));

      if (rx_buffer_.size() >= protocol::DESCRIPTOR_SIZE) {
        protocol::ResponseDescriptor desc{};
        protocol::parseDescriptor(rx_buffer_.data(), desc);
        rx_buffer_.erase(rx_buffer_.begin(),
                         rx_buffer_.begin() + static_cast<std::ptrdiff_t>(protocol::DESCRIPTOR_SIZE));

        if (desc.data_type != expected_type) {
          RCLCPP_ERROR(get_logger(), "Beklenmeyen cevap tipi: 0x%02X (beklenen: 0x%02X)",
                       desc.data_type, expected_type);
          return false;
        }
        payload_len = desc.payload_len;
        return true;
      }
    } else if (rx_buffer_.size() > 1) {
      // Son bayt 0xA5 olabilir; yalnızca onu sakla
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1);
    }

    if (fillRxBuffer(50) < 0) {
      return false;
    }
  }
  return false;
}

bool YdlidarDriverNode::readExact(uint8_t * buffer, size_t len, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (rx_buffer_.size() < len) {
    if (!running_.load() || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    if (fillRxBuffer(50) < 0) {
      return false;
    }
  }
  std::copy_n(rx_buffer_.begin(), len, buffer);
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(len));
  return true;
}

int YdlidarDriverNode::fillRxBuffer(int timeout_ms)
{
  uint8_t chunk[2048];
  const int n = serial_.readBytes(chunk, sizeof(chunk), timeout_ms);
  if (n > 0) {
    rx_buffer_.insert(rx_buffer_.end(), chunk, chunk + n);
  }
  return n;
}

// ============================================================================
// PAKET ÇÖZÜMLEME + TUR TESPİTİ
//   Tamponda 0x55 0xAA başlığı aranır; her tam paket protocol::parsePacket ile
//   noktalara çözülür. CT&0x01 (yeni turun ilk paketi) görülünce biriken
//   önceki tur yayınlanır.
// ============================================================================
void YdlidarDriverNode::parseBufferedPackets()
{
  size_t offset = 0;

  while (rx_buffer_.size() - offset >= protocol::PACKET_HEADER_SIZE) {
    const uint8_t * p = rx_buffer_.data() + offset;

    // Başlık hizası: 0x55 0xAA değilse 1 bayt kaydırıp yeniden senkronize ol
    if (p[0] != protocol::PH_BYTE1 || p[1] != protocol::PH_BYTE2) {
      ++offset;
      continue;
    }

    std::vector<protocol::ScanPoint> points;
    bool   ring_start = false;
    size_t consumed   = 0;
    const protocol::ParseResult r = protocol::parsePacket(
      p, rx_buffer_.size() - offset,
      static_cast<uint8_t>(sample_bytes_), static_cast<float>(distance_scale_mm_),
      angle_correction_, verify_checksum_,
      points, ring_start, consumed);

    if (r == protocol::PARSE_NEED_MORE) {
      break;  // paketin tamamı henüz gelmedi
    }
    if (r == protocol::PARSE_BAD) {
      offset += (consumed > 0) ? consumed : 1;
      ++bad_packet_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Bozuk paket atıldı, yeniden senkronize olunuyor (toplam: %llu)",
        static_cast<unsigned long long>(bad_packet_count_));
      continue;
    }

    // PARSE_OK
    if (ring_start && !scan_buffer_.empty()) {
      if (scan_buffer_.size() >= kMinPointsPerScan) {
        publishScan();
      }
      scan_buffer_.clear();
      scan_start_stamp_ = now();
    }

    scan_buffer_.insert(scan_buffer_.end(), points.begin(), points.end());

    if (scan_buffer_.size() > kMaxPointsPerScan) {
      RCLCPP_WARN(get_logger(), "Ring-start alınamadı, tarama tamponu sıfırlandı");
      scan_buffer_.clear();
    }

    offset += consumed;
  }

  rx_buffer_.erase(rx_buffer_.begin(),
                   rx_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
}

// ============================================================================
// LaserScan DÖNÜŞÜMÜ:
//  - Lidar açıyı SAAT YÖNÜNDE (CW) verir; REP-103 CCW ister -> invert_angle
//    true ise açı 360-θ ile aynalanır.
//  - Ölçümler sabit açısal ızgaraya (num_bins) oturtulur; boş hücreler +inf.
//  - Aynı hücreye düşen birden çok ölçümde EN YAKIN mesafe tutulur.
// ============================================================================
void YdlidarDriverNode::publishScan()
{
  const rclcpp::Time scan_end  = now();
  const double       scan_time = std::max((scan_end - scan_start_stamp_).seconds(), 1e-6);

  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp    = scan_start_stamp_;
  msg->header.frame_id = frame_id_;

  const double angle_increment = (2.0 * M_PI) / static_cast<double>(num_bins_);
  msg->angle_min       = 0.0f;
  msg->angle_max       = static_cast<float>(angle_increment * static_cast<double>(num_bins_ - 1));
  msg->angle_increment = static_cast<float>(angle_increment);
  msg->scan_time       = static_cast<float>(scan_time);
  msg->time_increment  = static_cast<float>(scan_time / static_cast<double>(scan_buffer_.size()));
  msg->range_min       = static_cast<float>(range_min_);
  msg->range_max       = static_cast<float>(range_max_);

  msg->ranges.assign(num_bins_, std::numeric_limits<float>::infinity());
  msg->intensities.assign(num_bins_, 0.0f);

  for (const auto & pt : scan_buffer_) {
    if (pt.distance_m <= 1e-4f) {
      continue;  // 0 => geçersiz ölçüm (yansıma alınamadı)
    }
    if (pt.angle_deg < 0.0f || pt.angle_deg >= 360.0f) {
      continue;
    }

    float angle_deg = pt.angle_deg;
    if (invert_angle_) {
      angle_deg = 360.0f - angle_deg;  // CW -> CCW
      if (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
      }
    }

    const size_t bin =
      static_cast<size_t>(std::lround(
        (angle_deg / 360.0f) * static_cast<float>(num_bins_))) % num_bins_;

    if (pt.distance_m < msg->ranges[bin]) {
      msg->ranges[bin]      = pt.distance_m;
      msg->intensities[bin] = static_cast<float>(pt.intensity);
    }
  }

  scan_pub_->publish(std::move(msg));
}

void YdlidarDriverNode::startMotor()
{
  // Yalnızca use_dtr_motor=true iken çağrılır. DTR LOW ile bazı adaptör
  // kartlarındaki motor sürücüsü etkinleşir.
  if (!serial_.setDtr(false)) {
    RCLCPP_WARN(get_logger(), "DTR temizlenemedi; motor dönmüyor olabilir");
  }
}

void YdlidarDriverNode::stopMotor()
{
  serial_.setDtr(true);
}

void YdlidarDriverNode::shutdownLidar()
{
  if (!serial_.isOpen()) {
    return;
  }
  sendCommand(protocol::CMD_STOP);
  std::this_thread::sleep_for(10ms);
  if (use_dtr_motor_) {
    stopMotor();
  }
  serial_.closePort();
  RCLCPP_INFO(get_logger(), "Lidar durduruldu, seri port kapatıldı");
}

}  // namespace ydlidar_tmini_driver

// ============================================================================
// GİRİŞ NOKTASI
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ydlidar_tmini_driver::YdlidarDriverNode>());
  rclcpp::shutdown();
  return 0;
}
