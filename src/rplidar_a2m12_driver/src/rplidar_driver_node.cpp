#include "rplidar_a2m12_driver/rplidar_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace rplidar_a2m12_driver
{

RplidarDriverNode::RplidarDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rplidar_a2m12_driver", options)
{
  // --- Parametreler (launch dosyasından veya CLI'dan geçersiz kılınabilir) ---
  port_      = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_rate_ = static_cast<uint32_t>(declare_parameter<int>("baud_rate", 256000));
  frame_id_  = declare_parameter<std::string>("frame_id", "laser_link");
  range_min_ = declare_parameter<double>("range_min", 0.15);
  range_max_ = declare_parameter<double>("range_max", 12.0);
  num_bins_  = static_cast<size_t>(declare_parameter<int>("num_bins", 720));

  // --- QoS: Sensor Data profili -> Best Effort + Volatile, derinlik 5.
  // Lidar gibi yüksek frekanslı akışlarda kayıp bir taramayı yeniden iletmek
  // anlamsızdır; Reliable QoS ağı tıkar ve FSM'leri geciktirir. ---
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  rx_buffer_.reserve(8192);
  scan_buffer_.reserve(4096);
  scan_start_stamp_ = now();

  RCLCPP_INFO(get_logger(),
              "RPLIDAR A2M12 sürücüsü başlatılıyor: port=%s baud=%u frame_id=%s",
              port_.c_str(), baud_rate_, frame_id_.c_str());

  // Tüm seri G/Ç ayrı thread'de: ROS executor'ı hiçbir zaman bloklanmaz
  running_.store(true);
  reader_thread_ = std::thread(&RplidarDriverNode::readerLoop, this);
}

RplidarDriverNode::~RplidarDriverNode()
{
  running_.store(false);
  if (reader_thread_.joinable()) {
    reader_thread_.join();  // shutdownLidar() thread çıkışında çağrılır
  }
}

// ============================================================================
// OKUYUCU THREAD ANA DÖNGÜSÜ
// Durum makinesi:
//   [BAĞLI DEĞİL] -> connectAndStartScan() -> [SENKRONİZE AKIŞ]
//   Akışta her 5 bayt bir ölçüm düğümüdür; bütünlük kontrolü tutmayan
//   baytlar tek tek atılarak yeniden senkronizasyon sağlanır.
// ============================================================================
void RplidarDriverNode::readerLoop()
{
  bool connected = false;

  while (running_.load() && rclcpp::ok()) {
    // --- DURUM 1: Bağlantı yok -> kur (başarısızsa 1 sn sonra tekrar dene) ---
    if (!connected) {
      connected = connectAndStartScan();
      if (!connected) {
        std::this_thread::sleep_for(1s);
        continue;
      }
      scan_buffer_.clear();
      scan_start_stamp_ = now();
    }

    // --- DURUM 2: Veri bekle (poll ile kernel'de uyuyarak; CPU harcamaz) ---
    const int n = fillRxBuffer(200);
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "Seri port okuma hatası; yeniden bağlanılacak");
      serial_.closePort();
      connected = false;
      continue;
    }
    if (n == 0) {
      continue;  // zaman aşımı: bu pencerede veri gelmedi
    }

    // --- DURUM 3: Tampondaki 5 baytlık ölçüm düğümlerini ayrıştır ---
    size_t offset = 0;
    while (rx_buffer_.size() - offset >= protocol::SCAN_NODE_SIZE) {
      protocol::ScanNode node;
      if (protocol::parseScanNode(rx_buffer_.data() + offset, node)) {
        offset += protocol::SCAN_NODE_SIZE;
        handleNode(node);
      } else {
        // Bütünlük kontrolü başarısız (S/!S veya C biti tutmadı):
        // 1 bayt kaydırarak yeniden senkronize ol, paketi çöpe at
        ++offset;
        ++corrupt_byte_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Bozuk ölçüm paketi atıldı, yeniden senkronize olunuyor (toplam: %llu bayt)",
          static_cast<unsigned long long>(corrupt_byte_count_));
      }
    }
    // İşlenen kısmı tampondan düşür
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
  }

  shutdownLidar();
}

// ============================================================================
// BAĞLANTI SIRASI: port aç -> STOP -> GET_HEALTH -> motoru döndür (DTR LOW)
//                  -> SCAN (0xA5 0x20) -> descriptor doğrula (tip 0x81, len 5)
// ============================================================================
bool RplidarDriverNode::connectAndStartScan()
{
  std::string error_msg;
  if (!serial_.openPort(port_, baud_rate_, error_msg)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Seri port açılamadı (%s): %s", port_.c_str(), error_msg.c_str());
    return false;
  }
  rx_buffer_.clear();

  // Cihaz önceki oturumdan tarama modunda kalmış olabilir: önce durdur,
  // eski akış artıklarını tampondan temizle
  if (!sendCommand(protocol::CMD_STOP)) {
    serial_.closePort();
    return false;
  }
  std::this_thread::sleep_for(50ms);
  serial_.flushInput();
  rx_buffer_.clear();

  if (!checkHealth()) {
    serial_.closePort();
    return false;
  }

  // Motoru döndür: DTR LOW -> USB adaptör kartındaki PWM devreye girer.
  // Nominal dönüş hızına (~10 Hz) oturması için 1 sn bekle; aksi halde ilk
  // turların açısal dağılımı bozuk olur.
  startMotor();
  std::this_thread::sleep_for(1000ms);

  // Start Scan komutu: 0xA5 0x20
  if (!sendCommand(protocol::CMD_SCAN)) {
    RCLCPP_ERROR(get_logger(), "SCAN komutu gönderilemedi");
    stopMotor();
    serial_.closePort();
    return false;
  }

  // Cevap descriptor'ı beklenir: A5 5A 05 00 00 40 81
  // (payload_len=5, send_mode=1 [sürekli akış], data_type=0x81)
  uint32_t payload_len = 0;
  if (!waitResponseDescriptor(protocol::ANS_TYPE_MEASUREMENT, payload_len, 2000) ||
      payload_len != protocol::SCAN_NODE_SIZE) {
    RCLCPP_ERROR(get_logger(), "SCAN cevap descriptor'ı doğrulanamadı");
    stopMotor();
    serial_.closePort();
    return false;
  }

  RCLCPP_INFO(get_logger(), "RPLIDAR taramaya başladı (%s @ %u baud)",
              port_.c_str(), baud_rate_);
  return true;
}

// ============================================================================
// GET_HEALTH: cihaz "Protection Stop" (status=2) durumundaysa SCAN komutunu
// yok sayar; bu durumda RESET gönderip yeniden bağlanmak gerekir.
// ============================================================================
bool RplidarDriverNode::checkHealth()
{
  if (!sendCommand(protocol::CMD_GET_HEALTH)) {
    return false;
  }

  uint32_t payload_len = 0;
  if (!waitResponseDescriptor(protocol::ANS_TYPE_DEVHEALTH, payload_len, 1000) ||
      payload_len != 3) {
    RCLCPP_ERROR(get_logger(),
                 "GET_HEALTH cevabı alınamadı (lidar bağlı mı? baud rate doğru mu?)");
    return false;
  }

  // Payload: [status(1)] [error_code_lo(1)] [error_code_hi(1)]
  uint8_t health[3] = {0, 0, 0};
  if (!readExact(health, sizeof(health), 500)) {
    RCLCPP_ERROR(get_logger(), "GET_HEALTH payload'ı zaman aşımına uğradı");
    return false;
  }

  const uint8_t  status     = health[0];
  const uint16_t error_code = static_cast<uint16_t>(health[1] | (health[2] << 8));

  if (status == protocol::HEALTH_STATUS_ERROR) {
    RCLCPP_ERROR(get_logger(),
                 "Lidar 'Protection Stop' durumunda (hata kodu: 0x%04X); RESET gönderiliyor",
                 error_code);
    sendCommand(protocol::CMD_RESET);
    // RESET sonrası cihaz yeniden başlar ve porta metinsel bir açılış mesajı
    // basar; tamamlanmasını bekleyip tamponu temizliyoruz
    std::this_thread::sleep_for(2000ms);
    serial_.flushInput();
    rx_buffer_.clear();
    return false;  // sonraki bağlantı denemesinde sağlık yeniden kontrol edilir
  }
  if (status == protocol::HEALTH_STATUS_WARNING) {
    RCLCPP_WARN(get_logger(), "Lidar sağlık durumu: WARNING (hata kodu: 0x%04X)", error_code);
  } else {
    RCLCPP_INFO(get_logger(), "Lidar sağlık durumu: OK");
  }
  return true;
}

bool RplidarDriverNode::sendCommand(uint8_t cmd, const uint8_t * payload, uint8_t payload_len)
{
  uint8_t packet[protocol::MAX_COMMAND_SIZE];
  const size_t len = protocol::buildCommand(cmd, payload, payload_len, packet);
  return serial_.writeBytes(packet, len);
}

// ============================================================================
// DESCRIPTOR ARAMA DURUM MAKİNESİ: asenkron bayt akışında 0xA5 0x5A çiftini
// arar; bulunca 7 baytlık descriptor'ı çözümleyip tipini doğrular.
// Descriptor öncesi gelen çöp baytlar atılır.
// ============================================================================
bool RplidarDriverNode::waitResponseDescriptor(uint8_t expected_type,
                                               uint32_t & payload_len, int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    // Tamponda senkronizasyon çiftini ara
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
      // Başlıktan önceki çöp baytları at
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
      // Başlık bulundu ama 7 bayt henüz tamamlanmadı: okumaya devam et
    } else if (rx_buffer_.size() > 1) {
      // Eşleşme yok: son bayt 0xA5 olabileceği için yalnızca onu sakla
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - 1);
    }

    if (fillRxBuffer(50) < 0) {
      return false;
    }
  }
  return false;
}

bool RplidarDriverNode::readExact(uint8_t * buffer, size_t len, int timeout_ms)
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

int RplidarDriverNode::fillRxBuffer(int timeout_ms)
{
  uint8_t chunk[1024];
  const int n = serial_.readBytes(chunk, sizeof(chunk), timeout_ms);
  if (n > 0) {
    rx_buffer_.insert(rx_buffer_.end(), chunk, chunk + n);
  }
  return n;
}

// ============================================================================
// 360 DERECE TAM TUR TESPİTİ: start flag'li düğüm, yeni turun İLK ölçümüdür.
// O ana kadar biriken noktalar bir önceki (tamamlanmış) turu oluşturur.
// ============================================================================
void RplidarDriverNode::handleNode(const protocol::ScanNode & node)
{
  if (node.start_flag && !scan_buffer_.empty()) {
    // İlk (eksik) turu yayınlama; tam turlar yeterli nokta içerir
    if (scan_buffer_.size() >= kMinNodesPerScan) {
      publishScan();
    }
    scan_buffer_.clear();
    scan_start_stamp_ = now();
  }

  scan_buffer_.push_back(node);

  // Güvenlik: start flag hiç gelmezse (donanım arızası) belleği sınırla
  if (scan_buffer_.size() > kMaxNodesPerScan) {
    RCLCPP_WARN(get_logger(), "Start flag alınamadı, tarama tamponu sıfırlandı");
    scan_buffer_.clear();
  }
}

// ============================================================================
// LaserScan DÖNÜŞÜMÜ:
//  - Lidar açıyı SAAT YÖNÜNDE (CW) verir; REP-103 saat yönünün tersini (CCW)
//    ister -> açı 360-θ ile aynalanır.
//  - Ölçümler sabit açısal ızgaraya (num_bins) oturtulur; boş hücreler +inf
//    kalır (LaserScan tekdüze angle_increment gerektirir, ham örnekleme ise
//    turdan tura hafifçe kayar).
//  - Aynı hücreye düşen birden çok ölçümde EN YAKIN mesafe tutulur
//    (engelden kaçınma için güvenli taraf).
// ============================================================================
void RplidarDriverNode::publishScan()
{
  const rclcpp::Time scan_end  = now();
  const double       scan_time = std::max((scan_end - scan_start_stamp_).seconds(), 1e-6);

  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp    = scan_start_stamp_;  // turun İLK ölçümünün zamanı
  msg->header.frame_id = frame_id_;          // "laser_link"

  const double angle_increment = (2.0 * M_PI) / static_cast<double>(num_bins_);
  msg->angle_min       = 0.0f;
  msg->angle_max       = static_cast<float>(angle_increment * static_cast<double>(num_bins_ - 1));
  msg->angle_increment = static_cast<float>(angle_increment);
  msg->scan_time       = static_cast<float>(scan_time);
  msg->time_increment  = static_cast<float>(scan_time / static_cast<double>(scan_buffer_.size()));
  msg->range_min       = static_cast<float>(range_min_);
  msg->range_max       = static_cast<float>(range_max_);

  // Ölçüm alınamayan yönler +inf: "bu yönde menzil içinde engel görülmedi"
  msg->ranges.assign(num_bins_, std::numeric_limits<float>::infinity());
  msg->intensities.assign(num_bins_, 0.0f);

  for (const auto & node : scan_buffer_) {
    // quality=0 veya distance=0: lidar yansıma alamamış -> geçersiz ölçüm
    if (node.quality == 0 || node.distance_m <= 1e-4f) {
      continue;
    }
    if (node.angle_deg < 0.0f || node.angle_deg >= 360.0f) {
      continue;  // bit hatasından sızmış anlamsız açı
    }

    // CW -> CCW dönüşümü (REP-103)
    float angle_ccw_deg = 360.0f - node.angle_deg;
    if (angle_ccw_deg >= 360.0f) {
      angle_ccw_deg -= 360.0f;
    }

    const size_t bin =
      static_cast<size_t>(std::lround(
        (angle_ccw_deg / 360.0f) * static_cast<float>(num_bins_))) % num_bins_;

    if (node.distance_m < msg->ranges[bin]) {
      msg->ranges[bin]      = node.distance_m;
      msg->intensities[bin] = static_cast<float>(node.quality);
    }
  }

  scan_pub_->publish(std::move(msg));
}

void RplidarDriverNode::startMotor()
{
  // DTR LOW: USB adaptör kartındaki motor sürücüsü (MOTOCTL) PWM üretmeye
  // başlar ve tarama kafası döner. Not: Linux open() çağrısı DTR'yi varsayılan
  // olarak HIGH bırakır, bu yüzden bu çağrı olmadan motor DÖNMEZ.
  if (!serial_.setDtr(false)) {
    RCLCPP_WARN(get_logger(), "DTR temizlenemedi; motor dönmüyor olabilir");
  }
}

void RplidarDriverNode::stopMotor()
{
  // DTR HIGH: PWM kesilir, motor durur
  serial_.setDtr(true);
}

void RplidarDriverNode::shutdownLidar()
{
  if (!serial_.isOpen()) {
    return;
  }
  // Düzgün kapanış: veri akışını kes, motoru durdur, portu bırak
  sendCommand(protocol::CMD_STOP);
  std::this_thread::sleep_for(10ms);
  stopMotor();
  serial_.closePort();
  RCLCPP_INFO(get_logger(), "Lidar durduruldu, seri port kapatıldı");
}

}  // namespace rplidar_a2m12_driver

// ============================================================================
// GİRİŞ NOKTASI
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // spin() yalnızca parametre servisi ve logging için döner; tüm seri G/Ç
  // düğümün kendi okuyucu thread'inde yürür. Ctrl+C -> spin döner ->
  // yıkıcı (destructor) thread'i durdurur -> STOP + motor kapatma yapılır.
  rclcpp::spin(std::make_shared<rplidar_a2m12_driver::RplidarDriverNode>());
  rclcpp::shutdown();
  return 0;
}
