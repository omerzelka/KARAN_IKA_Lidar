#pragma once

#include <cstddef>
#include <cstdint>

// ============================================================================
// Slamtec RPLIDAR seri protokolü — SDK'sız, sıfırdan implementasyon.
//
// İstek paketi : [0xA5][CMD]                              (payload'sız)
//                [0xA5][CMD][LEN][PAYLOAD...][CHECKSUM]   (payload'lı)
// Cevap paketi : önce 7 baytlık "Response Descriptor":
//                [0xA5][0x5A][LEN|MODE (4 bayt)][DATA_TYPE]
//                ardından LEN uzunluğunda veri (tek seferlik veya sürekli akış).
// Ölçüm düğümü : SCAN (0x20) modunda her ölçüm 5 bayttır (detay aşağıda).
// ============================================================================

namespace rplidar_a2m12_driver::protocol
{

// --- Senkronizasyon baytları ---
constexpr uint8_t SYNC_BYTE  = 0xA5;  // Tüm isteklerin ve cevap descriptor'ının ilk baytı
constexpr uint8_t SYNC_BYTE2 = 0x5A;  // Cevap descriptor'ının ikinci baytı

// --- Komutlar ---
constexpr uint8_t CMD_STOP       = 0x25;  // Taramayı durdur (cevap dönmez)
constexpr uint8_t CMD_RESET      = 0x40;  // Cihazı yeniden başlat (cevap dönmez, ~2 sn bekle)
constexpr uint8_t CMD_SCAN       = 0x20;  // Standart tarama modunu başlat (sürekli akış)
constexpr uint8_t CMD_GET_INFO   = 0x50;  // Cihaz bilgisi (model, firmware, seri no)
constexpr uint8_t CMD_GET_HEALTH = 0x52;  // Sağlık durumu sorgusu

// --- Cevap veri tipleri (descriptor'ın 7. baytı) ---
constexpr uint8_t ANS_TYPE_MEASUREMENT = 0x81;  // SCAN ölçüm akışı
constexpr uint8_t ANS_TYPE_DEVINFO     = 0x04;  // GET_INFO cevabı
constexpr uint8_t ANS_TYPE_DEVHEALTH   = 0x06;  // GET_HEALTH cevabı

// --- Boyutlar ---
constexpr size_t DESCRIPTOR_SIZE  = 7;    // 0xA5 0x5A + 4 bayt uzunluk/mod + 1 bayt tip
constexpr size_t SCAN_NODE_SIZE   = 5;    // Standart tarama modunda tek ölçümün boyutu
constexpr size_t MAX_COMMAND_SIZE = 259;  // 0xA5 + cmd + len + 255 payload + checksum

// --- GET_HEALTH durum kodları ---
constexpr uint8_t HEALTH_STATUS_OK      = 0;
constexpr uint8_t HEALTH_STATUS_WARNING = 1;
constexpr uint8_t HEALTH_STATUS_ERROR   = 2;  // "Protection Stop": RESET gerekir

// Cevap descriptor'ının çözümlenmiş hali
struct ResponseDescriptor
{
  uint32_t payload_len;  // 30 bitlik veri uzunluğu
  uint8_t  send_mode;    // 0: tek cevap, 1: sürekli akış (SCAN böyle)
  uint8_t  data_type;    // ANS_TYPE_* sabitlerinden biri
};

// Tek bir lidar ölçüm noktası (5 baytlık paketten çözümlenir)
struct ScanNode
{
  float   angle_deg;   // [0, 360) derece — lidar saat yönünde (CW) verir
  float   distance_m;  // metre; 0 => geçersiz ölçüm (yansıma alınamadı)
  uint8_t quality;     // 6 bitlik sinyal kalitesi (0-63); 0 => geçersiz
  bool    start_flag;  // true => yeni 360 derecelik turun ilk ölçümü
};

// ----------------------------------------------------------------------------
// Checksum: paketteki tüm baytların XOR'u. Slamtec protokolünde payload içeren
// komutların sonuna eklenir; alıcı taraf 0xA5'ten itibaren tüm baytları
// XOR'ladığında sonuç checksum baytına eşit olmalıdır.
// ----------------------------------------------------------------------------
inline uint8_t computeChecksum(const uint8_t * data, size_t len)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; ++i) {
    checksum ^= data[i];
  }
  return checksum;
}

// ----------------------------------------------------------------------------
// Komut paketi oluşturur. 'out' en az MAX_COMMAND_SIZE bayt olmalıdır.
// Dönüş değeri: pakete yazılan toplam bayt sayısı.
// ----------------------------------------------------------------------------
inline size_t buildCommand(uint8_t cmd, const uint8_t * payload,
                           uint8_t payload_len, uint8_t * out)
{
  size_t idx = 0;
  out[idx++] = SYNC_BYTE;
  out[idx++] = cmd;

  // Payload'lı komutlarda (örn. EXPRESS_SCAN 0x82) uzunluk + checksum eklenir
  if (payload != nullptr && payload_len > 0) {
    out[idx++] = payload_len;
    for (size_t i = 0; i < payload_len; ++i) {
      out[idx++] = payload[i];
    }
    // Checksum: o ana kadar yazılmış TÜM baytların (0xA5 dahil) XOR'u
    out[idx] = computeChecksum(out, idx);
    ++idx;
  }
  return idx;
}

// ----------------------------------------------------------------------------
// 7 baytlık cevap descriptor'ını çözümler. Çağıran taraf d[0]==0xA5 ve
// d[1]==0x5A eşleşmesini önceden doğrulamış olmalıdır.
//
// Bayt yerleşimi (little-endian):
//   d[2..5] : alt 30 bit = payload uzunluğu, üst 2 bit = gönderim modu
//   d[6]    : veri tipi
// ----------------------------------------------------------------------------
inline void parseDescriptor(const uint8_t * d, ResponseDescriptor & out)
{
  out.payload_len = static_cast<uint32_t>(d[2]) |
                    (static_cast<uint32_t>(d[3]) << 8) |
                    (static_cast<uint32_t>(d[4]) << 16) |
                    (static_cast<uint32_t>(d[5] & 0x3F) << 24);  // üst 2 bit moda ait
  out.send_mode = static_cast<uint8_t>(d[5] >> 6);
  out.data_type = d[6];
}

// ----------------------------------------------------------------------------
// 5 baytlık standart tarama ölçüm paketini bit kaydırma ile çözümler.
//
// Bit yerleşimi:
//   p[0] : bit0 = S (start flag), bit1 = !S (S'in tersi), bit2-7 = quality (6 bit)
//   p[1] : bit0 = C (kontrol biti, HER ZAMAN 1), bit1-7 = angle_q6'nın alt 7 biti
//   p[2] : angle_q6'nın üst 8 biti      -> açı    = angle_q6 / 64.0   [derece]
//   p[3] : distance_q2'nin alt 8 biti
//   p[4] : distance_q2'nin üst 8 biti   -> mesafe = distance_q2 / 4.0 [mm]
//
// Bütünlük kontrolü (standart modun paket doğrulaması):
//   1) S ile !S birbirinin tersi olmalı (aynıysa bayt akışı kaymış demektir)
//   2) C biti 1 olmalı
// Sağlanmazsa paket çöpe atılır, akış 1 bayt kaydırılarak yeniden senkronize edilir.
// ----------------------------------------------------------------------------
inline bool parseScanNode(const uint8_t * p, ScanNode & out)
{
  const uint8_t start_bit          = p[0] & 0x01;         // S
  const uint8_t inverted_start_bit = (p[0] >> 1) & 0x01;  // !S

  if (start_bit == inverted_start_bit) {
    return false;  // S ^ !S == 0 -> bozuk paket
  }
  if ((p[1] & 0x01) == 0) {
    return false;  // Kontrol biti 0 -> bozuk paket
  }

  out.start_flag = (start_bit == 1);
  out.quality    = static_cast<uint8_t>(p[0] >> 2);  // üst 6 bit

  // Açı: 14 bitlik sabit noktalı değer (Q6 formatı: değer/64 = derece)
  const uint16_t angle_q6 =
    static_cast<uint16_t>((static_cast<uint16_t>(p[2]) << 7) | (p[1] >> 1));
  out.angle_deg = static_cast<float>(angle_q6) / 64.0f;

  // Mesafe: 16 bitlik sabit noktalı değer (Q2 formatı: değer/4 = mm)
  const uint16_t distance_q2 =
    static_cast<uint16_t>(p[3] | (static_cast<uint16_t>(p[4]) << 8));
  out.distance_m = (static_cast<float>(distance_q2) / 4.0f) / 1000.0f;  // mm -> m

  return true;
}

}  // namespace rplidar_a2m12_driver::protocol
