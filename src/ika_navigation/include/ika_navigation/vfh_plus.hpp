#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ika_navigation
{

// ============================================================================
// VFH+ (Vector Field Histogram Plus) — engelden kaçınma yön seçici
//
// Borenstein & Koren'in VFH'inin Ulrich & Borenstein tarafından geliştirilmiş
// hali. Lidar taramasından bir POLAR HİSTOGRAM üretir ve tankın güvenle
// gidebileceği yönü seçer. Trafik konileri / kayar bariyerler gibi engelleri
// robot yarıçapı + güvenlik payı kadar "şişirerek" dar geçitlerde bile
// çarpışmasız yön bulur.
//
// Adımlar:
//   1) Birincil polar histogram: her sektöre engel yoğunluğu (yakın engel ->
//      büyük değer). Her ışın, robot yarıçapı için γ = asin((r+d_s)/d) kadar
//      açısal olarak genişletilir.
//   2) İkili histogram: histerezis eşikleriyle (tau_low/tau_high) sektörler
//      "serbest (0)" / "engelli (1)" olarak sınıflanır — gürültüde titremez.
//   3) Aday yönler: serbest bölgelerin (vadi) kenarlarından üretilir; geniş
//      vadilerde kenara yakın iki aday, dar vadilerde orta aday.
//   4) Maliyet fonksiyonu: g(c) = μ1·Δ(c,hedef) + μ2·Δ(c,yön) + μ3·Δ(c,önceki)
//      en düşük maliyetli aday seçilir (yumuşak, salınımsız sürüş).
//
// Açı konvansiyonu: robot çerçevesinde 0 rad = ileri (+x), CCW pozitif.
// ============================================================================
class VFHPlus
{
public:
  struct Params
  {
    int    num_sectors      = 72;    // histogram çözünürlüğü (360/72 = 5°)
    double robot_radius     = 0.35;  // tankın yarıçapı (m)
    double safety_distance  = 0.25;  // ek güvenlik payı (m)
    double max_range        = 4.0;   // bu mesafeden uzak engel dikkate alınmaz
    double threshold_low    = 3.0;   // histerezis alt eşiği
    double threshold_high   = 8.0;   // histerezis üst eşiği
    // Vadi genişlik eşiği: bu sektör sayısından geniş açıklık "geniş" sayılır
    int    wide_valley      = 8;     // ~40°
    // Maliyet ağırlıkları (mu1 > mu2 + mu3 önerilir: hedefe yönelim baskın)
    double mu1_target       = 5.0;   // hedef yönüne yakınlık
    double mu2_current      = 2.0;   // mevcut yöne (ani dönüşten kaçın)
    double mu3_previous     = 2.0;   // önceki seçime (salınımı bastır)
    // Histogram yoğunluğu a - b*d formülü için: a = b*max_range olacak şekilde
    double density_a        = 1.0;
  };

  struct Result
  {
    bool   valid;             // güvenli bir yön bulundu mu?
    double steering_angle;    // seçilen yön (robot çerçevesi, rad, 0=ileri)
    bool   blocked;           // tüm ön yönler kapalı mı? (dur / geri manevra)
    double nearest_obstacle;  // en yakın engel mesafesi (hız ölçekleme için)
  };

  VFHPlus();
  explicit VFHPlus(const Params & params);

  // Bir LaserScan taramasından güvenli yön hesapla.
  //   ranges         : mesafe ölçümleri (m); inf/nan/menzil dışı yok sayılır
  //   angle_min      : ilk ışının açısı (rad)
  //   angle_increment: ışınlar arası açı (rad)
  //   target_angle   : gitmek istenen yön (robot çerçevesi, rad) — Stanley'den
  Result computeSteering(const std::vector<float> & ranges,
                         double angle_min,
                         double angle_increment,
                         double target_angle);

  const std::vector<double> & polarHistogram() const { return polar_hist_; }
  const std::vector<uint8_t> & binaryHistogram() const { return binary_hist_; }

private:
  Params               params_;
  std::vector<double>  polar_hist_;    // birincil histogram
  std::vector<uint8_t> binary_hist_;   // ikili histogram (histerezisli, kalıcı)
  double               prev_direction_{0.0};
  bool                 has_prev_{false};

  int    angleToSector(double angle) const;   // rad -> sektör indeksi
  double sectorToAngle(int sector) const;     // sektör -> merkez açı (rad)
};

}  // namespace ika_navigation
