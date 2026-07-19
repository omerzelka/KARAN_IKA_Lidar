#include "ika_navigation/vfh_plus.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

namespace ika_navigation
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

// Açıyı [-pi, pi) aralığına indirger
double wrapPi(double a)
{
  while (a >= M_PI)  { a -= kTwoPi; }
  while (a < -M_PI)  { a += kTwoPi; }
  return a;
}

// İki açı arasındaki mutlak fark (sektör cinsinden değil, rad)
double angularDistance(double a, double b)
{
  return std::fabs(wrapPi(a - b));
}
}  // namespace

VFHPlus::VFHPlus()
: VFHPlus(Params{})
{
}

VFHPlus::VFHPlus(const Params & params)
: params_(params)
{
  polar_hist_.assign(static_cast<size_t>(params_.num_sectors), 0.0);
  binary_hist_.assign(static_cast<size_t>(params_.num_sectors), 0);
}

int VFHPlus::angleToSector(double angle) const
{
  // angle [-pi,pi) -> [0, 2pi) -> sektör
  double a = angle;
  while (a < 0.0)      { a += kTwoPi; }
  while (a >= kTwoPi)  { a -= kTwoPi; }
  int s = static_cast<int>(a / kTwoPi * params_.num_sectors);
  if (s >= params_.num_sectors) { s = 0; }
  return s;
}

double VFHPlus::sectorToAngle(int sector) const
{
  const double sector_width = kTwoPi / params_.num_sectors;
  return wrapPi((sector + 0.5) * sector_width);
}

VFHPlus::Result VFHPlus::computeSteering(const std::vector<float> & ranges,
                                         double angle_min,
                                         double angle_increment,
                                         double target_angle)
{
  const int    N            = params_.num_sectors;
  const double sector_width = kTwoPi / N;
  const double enlarge_r    = params_.robot_radius + params_.safety_distance;
  // a - b*d histogram formülü: d = max_range'de yoğunluk 0 olacak şekilde
  const double b = params_.density_a / std::max(params_.max_range, 1e-3);

  // --- 1) BİRİNCİL POLAR HİSTOGRAM ---
  std::fill(polar_hist_.begin(), polar_hist_.end(), 0.0);
  double nearest = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < ranges.size(); ++i) {
    const float d = ranges[i];
    if (!std::isfinite(d) || d <= 0.0f || d > params_.max_range) {
      continue;  // geçersiz / menzil dışı ölçüm
    }
    if (d < nearest) {
      nearest = d;
    }

    const double beta = angle_min + static_cast<double>(i) * angle_increment;  // ışın açısı

    // Engel yoğunluğu: yakın engel -> büyük (a - b*d)
    const double magnitude = params_.density_a - b * d;
    if (magnitude <= 0.0) {
      continue;
    }

    // Robot yarıçapı için açısal genişletme: gamma = asin((r+d_s)/d)
    // d < enlarge_r ise engel zaten robotun üstünde: tam genişletme (pi)
    double gamma;
    if (d <= enlarge_r) {
      gamma = M_PI_2;  // en az bir çeyrek; çok yakın engelde geniş blok
    } else {
      gamma = std::asin(std::min(1.0, enlarge_r / d));
    }

    // [beta-gamma, beta+gamma] aralığındaki tüm sektörlere yoğunluğu ekle
    const int span = static_cast<int>(std::ceil(gamma / sector_width));
    const int center = angleToSector(beta);
    for (int k = -span; k <= span; ++k) {
      int s = (center + k) % N;
      if (s < 0) { s += N; }
      polar_hist_[static_cast<size_t>(s)] += magnitude;
    }
  }

  // --- 2) İKİLİ HİSTOGRAM (histerezis) ---
  if (binary_hist_.size() != static_cast<size_t>(N)) {
    binary_hist_.assign(static_cast<size_t>(N), 0);
  }
  for (int s = 0; s < N; ++s) {
    const double v = polar_hist_[static_cast<size_t>(s)];
    if (v > params_.threshold_high) {
      binary_hist_[static_cast<size_t>(s)] = 1;   // kesin engelli
    } else if (v < params_.threshold_low) {
      binary_hist_[static_cast<size_t>(s)] = 0;   // kesin serbest
    }
    // ara bölge: önceki değeri koru (titremeyi engeller)
  }

  // --- 3) ADAY YÖNLER ---
  // Serbest sektörlerden aday açılar üret. Geniş vadilerde kenardan
  // (robot_radius kadar) içeride iki aday + varsa hedef; dar vadide orta.
  std::vector<double> candidates;
  candidates.reserve(static_cast<size_t>(N));

  // Tümü serbest mi? (açık alan) -> doğrudan hedefe git
  bool all_free = std::all_of(binary_hist_.begin(), binary_hist_.end(),
                              [](uint8_t x) { return x == 0; });
  if (all_free) {
    Result r;
    r.valid            = true;
    r.steering_angle   = wrapPi(target_angle);
    r.blocked          = false;
    r.nearest_obstacle = nearest;
    prev_direction_    = r.steering_angle;
    has_prev_          = true;
    return r;
  }

  // Vadi kenarlarını bul: dairesel dizide serbest sektör kümelerini tara.
  // Bir başlangıç engelli sektör bul ki sarma sınırını doğru ele alalım.
  int start = -1;
  for (int s = 0; s < N; ++s) {
    if (binary_hist_[static_cast<size_t>(s)] == 1) { start = s; break; }
  }

  if (start < 0) {
    // (all_free zaten ele alındı; buraya düşmemeli) güvenlik için hedefe git
    Result r{true, wrapPi(target_angle), false, nearest};
    prev_direction_ = r.steering_angle;
    has_prev_ = true;
    return r;
  }

  // start engelliden başlayarak daireyi dolaş, serbest blokları çıkar
  int i = 0;
  while (i < N) {
    int idx = (start + i) % N;
    if (binary_hist_[static_cast<size_t>(idx)] == 0) {
      // serbest blok başladı
      int block_start = idx;
      int len = 0;
      while (i < N && binary_hist_[static_cast<size_t>((start + i) % N)] == 0) {
        ++len;
        ++i;
      }
      int block_end = (start + i - 1) % N;  // son serbest sektör

      if (len >= params_.wide_valley) {
        // Geniş vadi: her iki kenardan wide_valley/2 içeride birer aday
        const int margin = params_.wide_valley / 2;
        int cand_r = (block_start + margin) % N;
        int cand_l = (block_end - margin + N) % N;
        candidates.push_back(sectorToAngle(cand_r));
        candidates.push_back(sectorToAngle(cand_l));
        // Hedef yön bu vadinin içindeyse doğrudan hedefi de aday yap
        int tsec = angleToSector(target_angle);
        // vadi içinde mi? (dairesel aralık testi)
        bool inside = false;
        for (int k = 0; k < len; ++k) {
          if ((block_start + k) % N == tsec) { inside = true; break; }
        }
        if (inside) {
          candidates.push_back(wrapPi(target_angle));
        }
      } else {
        // Dar vadi: yalnızca ortası (tek geçiş noktası)
        int mid = (block_start + len / 2) % N;
        candidates.push_back(sectorToAngle(mid));
      }
    } else {
      ++i;
    }
  }

  // --- 4) MALİYET FONKSİYONU ile en iyi adayı seç ---
  if (candidates.empty()) {
    // Hiç serbest yön yok: yol kapalı
    Result r;
    r.valid            = false;
    r.steering_angle   = has_prev_ ? prev_direction_ : 0.0;
    r.blocked          = true;
    r.nearest_obstacle = nearest;
    return r;
  }

  const double prev = has_prev_ ? prev_direction_ : target_angle;
  double best_cost  = std::numeric_limits<double>::infinity();
  double best_angle = candidates.front();

  for (double c : candidates) {
    const double cost = params_.mu1_target  * angularDistance(c, target_angle) +
                        params_.mu2_current * angularDistance(c, 0.0) +          // 0 = mevcut ileri yön
                        params_.mu3_previous * angularDistance(c, prev);
    if (cost < best_cost) {
      best_cost  = cost;
      best_angle = c;
    }
  }

  Result r;
  r.valid            = true;
  r.steering_angle   = wrapPi(best_angle);
  r.blocked          = false;
  r.nearest_obstacle = nearest;

  prev_direction_ = r.steering_angle;
  has_prev_       = true;
  return r;
}

}  // namespace ika_navigation
