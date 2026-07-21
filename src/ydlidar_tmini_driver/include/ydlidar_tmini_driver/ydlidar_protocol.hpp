#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// ============================================================================
// YDLIDAR T-mini Plus seri protokolü — SDK'sız, sıfırdan implementasyon.
//
// RPLIDAR'dan (Slamtec) FARKLARI:
//   * Ölçümler tek tek 5 baytlık düğüm halinde DEĞİL, çok noktalı VERİ PAKETLERİ
//     halinde gelir. Her paket bir başlık (0x55 0xAA) + N örnek içerir.
//   * Açı, paketin ilk (FSA) ve son (LSA) örnek açısı arasında lineer
//     interpolasyonla hesaplanır. Her nokta için ayrı açı byte'ı yoktur.
//   * Komut kodları farklıdır (SCAN=0x60, STOP=0x65, ... ; SYNC yine 0xA5).
//
// AYNI KALAN:
//   * İstek : [0xA5][CMD]
//   * Cevap descriptor'ı (7 bayt): [0xA5][0x5A][LEN|MODE (4 bayt)][DATA_TYPE]
//     -> Bu Slamtec ile birebir aynı yapıdadır, aynı parse mantığı kullanılır.
//
// Seri ayarları (T-mini Plus): 230400 baud, 8N1.
//
// ----------------------------------------------------------------------------
// DONANIMDA DOĞRULANMASI GEREKEN SABİTLER (parametreye bağlandı, driver_node'da):
//   - kDistanceScaleMmDefault : ham örnek -> mm dönüşümü (standart = /4).
//   - sample_bytes            : örnek başına bayt (standart/mesafe-only = 2,
//                               yoğunluklu [intensity] varyant = 3).
//   - enable_angle_correction : üçgenleme (triangulation) açı düzeltmesi.
//                               T-mini Plus ToF olduğundan VARSAYILAN KAPALI.
// İlk çalıştırmada tur başına nokta sayısı loglanır; ~360-720 civarı değilse
// bu sabitleri kontrol et (ACIKLAMA dosyasına bak).
// ============================================================================

namespace ydlidar_tmini_driver::protocol
{

// --- İstek/cevap senkronizasyon baytları ---
constexpr uint8_t SYNC_BYTE  = 0xA5;  // Tüm isteklerin ve cevap descriptor'ının ilk baytı
constexpr uint8_t SYNC_BYTE2 = 0x5A;  // Cevap descriptor'ının ikinci baytı

// --- Komutlar (YDLIDAR) ---
constexpr uint8_t CMD_SCAN            = 0x60;  // Tarama başlat (sürekli akış)
constexpr uint8_t CMD_FORCE_SCAN      = 0x61;  // Zorla tarama (sağlık kontrolünü atlar)
constexpr uint8_t CMD_STOP            = 0x65;  // Taramayı durdur
constexpr uint8_t CMD_RESET           = 0x80;  // Yazılımsal yeniden başlatma (soft restart)
constexpr uint8_t CMD_GET_DEVICE_INFO = 0x90;  // Cihaz bilgisi (model, firmware, seri no)
constexpr uint8_t CMD_GET_HEALTH      = 0x91;  // Sağlık durumu

// --- Cevap veri tipleri (descriptor'ın 7. baytı) ---
constexpr uint8_t ANS_TYPE_MEASUREMENT = 0x81;  // SCAN ölçüm akışı
constexpr uint8_t ANS_TYPE_DEVINFO     = 0x04;  // GET_DEVICE_INFO cevabı
constexpr uint8_t ANS_TYPE_DEVHEALTH   = 0x06;  // GET_HEALTH cevabı

// --- Boyutlar ---
constexpr size_t DESCRIPTOR_SIZE = 7;   // 0xA5 0x5A + 4 bayt uzunluk/mod + 1 bayt tip

// --- Veri paketi (node_package) başlığı ---
// package_Head = 0x55AA, little-endian gönderilir -> WIRE SIRASI: 0xAA sonra 0x55.
// (YDLIDAR SDK waitPackage(): 1. bayt == PH&0xFF == 0xAA, 2. bayt == PH>>8 == 0x55.)
// Bayt yerleşimi:
//   [0..1] PH  = 0xAA 0x55
//   [2]    CT  : paket tipi. bit0 = 1 => turun İLK ("zero") paketi (yeni 360°)
//   [3]    LSN : bu paketteki örnek (nokta) sayısı
//   [4..5] FSA : ilk örnek açısı (ham); bit0 = C kontrol biti
//   [6..7] LSA : son örnek açısı (ham); bit0 = C kontrol biti
//   [8..9] CS  : sağlama kodu (tüm 16-bit kelimelerin XOR'u, CS hariç)
//   [10..] Si  : örnek verileri (standart modda örnek başına 2 bayt = mesafe)
constexpr uint8_t  PH_BYTE1        = 0xAA;  // paket başlığı 1. bayt (wire sırası)
constexpr uint8_t  PH_BYTE2        = 0x55;  // paket başlığı 2. bayt
constexpr size_t   PACKET_HEADER_SIZE = 10; // PH..CS toplam sabit başlık uzunluğu
constexpr uint8_t  CT_RING_START   = 0x01;  // CT & 0x01 => yeni turun ilk paketi

// Ham örnek -> mm ölçek: standart YDLIDAR formatında mesafe = ham / 4.
constexpr float kDistanceScaleMmDefault = 0.25f;  // mm / ham_birim  (1/4)

// Üçgenleme (triangulation) açı düzeltmesi katsayıları — YALNIZCA üçgenlemeli
// modellerde (X4/G serisi). T-mini Plus ToF olduğu için varsayılan devre dışı.
constexpr float kAngCorrA = 21.8f;
constexpr float kAngCorrB = 155.3f;

// Çözümlenmiş cevap descriptor'ı (Slamtec ile aynı yapı)
struct ResponseDescriptor
{
  uint32_t payload_len;  // alt 30 bit: veri uzunluğu
  uint8_t  send_mode;    // üst 2 bit: 0 = tek cevap, 1 = sürekli akış (SCAN)
  uint8_t  data_type;    // ANS_TYPE_* sabitlerinden biri
};

// Tek bir çözümlenmiş lidar noktası
struct ScanPoint
{
  float    angle_deg;   // [0, 360) derece (lidarın ham çıktısı; CW)
  float    distance_m;  // metre; 0 => geçersiz ölçüm
  uint16_t intensity;   // sinyal yoğunluğu (mesafe-only modda 0)
};

// ----------------------------------------------------------------------------
// 7 baytlık cevap descriptor'ını çözümler. Çağıran d[0]==0xA5 && d[1]==0x5A
// eşleşmesini önceden doğrulamış olmalıdır. (Slamtec ile birebir aynı.)
// ----------------------------------------------------------------------------
inline void parseDescriptor(const uint8_t * d, ResponseDescriptor & out)
{
  out.payload_len = static_cast<uint32_t>(d[2]) |
                    (static_cast<uint32_t>(d[3]) << 8) |
                    (static_cast<uint32_t>(d[4]) << 16) |
                    (static_cast<uint32_t>(d[5] & 0x3F) << 24);
  out.send_mode = static_cast<uint8_t>(d[5] >> 6);
  out.data_type = d[6];
}

// ----------------------------------------------------------------------------
// Ham açı alanı (FSA/LSA) -> derece. Bit0 kontrol biti (C) atılır, kalan
// 15 bit Q6 formatındadır (değer/64 = derece).
// ----------------------------------------------------------------------------
inline float rawAngleToDeg(uint16_t raw)
{
  return static_cast<float>(raw >> 1) / 64.0f;
}

// ----------------------------------------------------------------------------
// Bir tam veri paketini çözümler.
//
// Girdi:
//   buf/len            : PH'den başlayan bayt tamponu (en az 'len' bayt geçerli)
//   sample_bytes       : örnek başına bayt (2 = mesafe-only, 3 = yoğunluklu)
//   dist_scale_mm      : ham -> mm ölçek (varsayılan 0.25)
//   angle_correction   : true ise üçgenleme açı düzeltmesi uygula (ToF'ta false)
//
// Çıktı:
//   out_points         : çözümlenen noktalar (append edilir)
//   out_ring_start     : bu paket yeni turun ilk paketi mi (CT & 0x01)
//   out_consumed       : tüketilen toplam bayt (paketin tam uzunluğu)
//
// Dönüş:
//   PARSE_OK        : paket çözümlendi
//   PARSE_NEED_MORE : tamponda paketin tamamı yok (daha fazla bayt bekle)
//   PARSE_BAD       : bozuk paket (başlık yok / CS tutmadı / mantıksız LSN)
// ----------------------------------------------------------------------------
enum ParseResult { PARSE_OK, PARSE_NEED_MORE, PARSE_BAD };

inline ParseResult parsePacket(const uint8_t * buf, size_t len,
                               uint8_t sample_bytes, float dist_scale_mm,
                               bool angle_correction, bool verify_checksum,
                               std::vector<ScanPoint> & out_points,
                               bool & out_ring_start, size_t & out_consumed)
{
  // Başlık için en az sabit 10 bayt gerekli
  if (len < PACKET_HEADER_SIZE) {
    return PARSE_NEED_MORE;
  }
  if (buf[0] != PH_BYTE1 || buf[1] != PH_BYTE2) {
    return PARSE_BAD;  // PH eşleşmedi -> çağıran 1 bayt kaydırıp resync yapmalı
  }

  const uint8_t  ct  = buf[2];
  const uint8_t  lsn = buf[3];
  const uint16_t fsa = static_cast<uint16_t>(buf[4] | (buf[5] << 8));
  const uint16_t lsa = static_cast<uint16_t>(buf[6] | (buf[7] << 8));
  const uint16_t cs  = static_cast<uint16_t>(buf[8] | (buf[9] << 8));

  const size_t packet_len = PACKET_HEADER_SIZE +
                            static_cast<size_t>(lsn) * sample_bytes;
  if (len < packet_len) {
    return PARSE_NEED_MORE;  // örnekler henüz tam gelmedi
  }

  // Sağlama: paketteki tüm 16-bit kelimelerin XOR'u CS'e eşit olmalı.
  // (XOR değişmeli olduğu için sıralama önemsizdir; CS'in kendisi hariç.)
  if (verify_checksum) {
    uint16_t chk = 0;
    chk ^= static_cast<uint16_t>(PH_BYTE1 | (PH_BYTE2 << 8));  // PH = 0xAA55
    chk ^= static_cast<uint16_t>(ct | (lsn << 8));            // CT (low) + LSN (high)
    chk ^= fsa;
    chk ^= lsa;
    for (uint8_t i = 0; i < lsn; ++i) {
      const size_t o = PACKET_HEADER_SIZE + static_cast<size_t>(i) * sample_bytes;
      // Mesafe kelimesi örneğin son 2 baytındadır (2B: [0..1], 3B: [1..2])
      const size_t d = (sample_bytes == 3) ? o + 1 : o;
      chk ^= static_cast<uint16_t>(buf[d] | (buf[d + 1] << 8));
    }
    if (chk != cs) {
      out_consumed = packet_len;  // paketi at ama akışı bu paket boyu kadar ilerlet
      return PARSE_BAD;
    }
  }

  out_ring_start = (ct & CT_RING_START) != 0;
  out_consumed   = packet_len;

  const float fsa_deg = rawAngleToDeg(fsa);
  const float lsa_deg = rawAngleToDeg(lsa);
  float diff = lsa_deg - fsa_deg;
  if (diff < 0.0f) {
    diff += 360.0f;  // tur sonundan başına sarma
  }
  const float per = (lsn > 1) ? diff / static_cast<float>(lsn - 1) : 0.0f;

  for (uint8_t i = 0; i < lsn; ++i) {
    const size_t o = PACKET_HEADER_SIZE + static_cast<size_t>(i) * sample_bytes;

    uint16_t raw_dist;
    uint16_t intensity = 0;
    if (sample_bytes == 3) {
      // Yoğunluklu varyant: [intensity(1B)][distance(2B)]
      intensity = buf[o];
      raw_dist  = static_cast<uint16_t>(buf[o + 1] | (buf[o + 2] << 8));
    } else {
      raw_dist  = static_cast<uint16_t>(buf[o] | (buf[o + 1] << 8));
    }

    const float dist_mm = static_cast<float>(raw_dist) * dist_scale_mm;

    float angle_deg = fsa_deg + per * static_cast<float>(i);

    // Üçgenleme açı düzeltmesi (yalnızca ToF-olmayan modeller)
    if (angle_correction && dist_mm > 1e-3f) {
      const float corr = std::atan(kAngCorrA * (kAngCorrB - dist_mm) /
                                   (kAngCorrB * dist_mm));
      angle_deg += corr * 180.0f / static_cast<float>(M_PI);
    }

    // [0, 360) aralığına indir
    angle_deg = std::fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f) {
      angle_deg += 360.0f;
    }

    ScanPoint p;
    p.angle_deg  = angle_deg;
    p.distance_m = dist_mm / 1000.0f;
    p.intensity  = intensity;
    out_points.push_back(p);
  }

  return PARSE_OK;
}

}  // namespace ydlidar_tmini_driver::protocol
