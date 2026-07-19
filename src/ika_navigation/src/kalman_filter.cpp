#include "ika_navigation/kalman_filter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ika_navigation
{

// ----------------------------------------------------------------------------
// KalmanFilter2D
// ----------------------------------------------------------------------------
KalmanFilter2D::KalmanFilter2D(double process_noise, double measurement_noise)
: q_(process_noise), r_(measurement_noise)
{
}

void KalmanFilter2D::init(double px, double py)
{
  x_ = {px, py, 0.0, 0.0};

  // Başlangıç kovaryansı: konumdan bir miktar, hızdan çok emin değiliz
  P_.fill(0.0);
  P_[0]  = r_;      // Pxx
  P_[5]  = r_;      // Pyy
  P_[10] = 10.0;    // Pvxvx (hız başta bilinmiyor -> büyük belirsizlik)
  P_[15] = 10.0;    // Pvyvy
  initialized_ = true;
}

// Tahmin adımı: sabit hız modeli
//   F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]
void KalmanFilter2D::predict(double dt)
{
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  // x' = F x  (yalnızca konum, hız*dt kadar ilerler)
  x_[0] += x_[2] * dt;
  x_[1] += x_[3] * dt;

  // P' = F P F^T + Q
  // F P F^T'yi açık yazmak yerine 4x4 çarpımı elle uyguluyoruz.
  // Süreç gürültüsü Q: ivmeye bağlı sürekli-zaman beyaz gürültü ayrıklaştırması.
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  // Q blokları (tek eksen için); q_ spektral yoğunluk
  const double q11 = dt4 / 4.0 * q_;
  const double q13 = dt3 / 2.0 * q_;
  const double q33 = dt2 * q_;

  // Geçici P kopyası
  std::array<double, 16> P = P_;
  auto P_at = [&P](int i, int j) -> double & { return P[i * 4 + j]; };
  auto Pn   = [this](int i, int j) -> double & { return P_[i * 4 + j]; };

  // F P F^T açık formülü (F yalnızca satır0+=dt*satır2, satır1+=dt*satır3):
  // Yeni P = F P F^T. Aşağıda blok-blok hesaplanır.
  // x ekseni (indeks 0=px, 2=vx), y ekseni (1=py, 3=vy) bağımsız gibi işlenir
  // ama çapraz terimler de korunur.
  std::array<double, 16> FP{};
  auto FP_at = [&FP](int i, int j) -> double & { return FP[i * 4 + j]; };

  // FP = F * P
  for (int j = 0; j < 4; ++j) {
    FP_at(0, j) = P_at(0, j) + dt * P_at(2, j);
    FP_at(1, j) = P_at(1, j) + dt * P_at(3, j);
    FP_at(2, j) = P_at(2, j);
    FP_at(3, j) = P_at(3, j);
  }
  // P_ = FP * F^T
  for (int i = 0; i < 4; ++i) {
    Pn(i, 0) = FP_at(i, 0) + dt * FP_at(i, 2);
    Pn(i, 1) = FP_at(i, 1) + dt * FP_at(i, 3);
    Pn(i, 2) = FP_at(i, 2);
    Pn(i, 3) = FP_at(i, 3);
  }

  // + Q (px-vx ve py-vy blokları)
  Pn(0, 0) += q11;  Pn(0, 2) += q13;
  Pn(2, 0) += q13;  Pn(2, 2) += q33;
  Pn(1, 1) += q11;  Pn(1, 3) += q13;
  Pn(3, 1) += q13;  Pn(3, 3) += q33;
}

// Güncelleme adımı: H = [1 0 0 0; 0 1 0 0]
void KalmanFilter2D::update(double px, double py)
{
  if (!initialized_) {
    init(px, py);
    return;
  }

  auto P = [this](int i, int j) -> double & { return P_[i * 4 + j]; };

  // İnovasyon: y = z - H x
  const double y0 = px - x_[0];
  const double y1 = py - x_[1];

  // S = H P H^T + R  (2x2)
  const double S00 = P(0, 0) + r_;
  const double S01 = P(0, 1);
  const double S10 = P(1, 0);
  const double S11 = P(1, 1) + r_;

  // S^-1
  const double det = S00 * S11 - S01 * S10;
  if (std::fabs(det) < 1e-12) {
    return;  // tekil; bu güncellemeyi atla
  }
  const double invdet = 1.0 / det;
  const double Si00 =  S11 * invdet;
  const double Si01 = -S01 * invdet;
  const double Si10 = -S10 * invdet;
  const double Si11 =  S00 * invdet;

  // K = P H^T S^-1  (4x2).  P H^T = P'in ilk iki sütunu
  std::array<double, 8> K{};  // [4x2]
  for (int i = 0; i < 4; ++i) {
    const double ph0 = P(i, 0);  // (P H^T) sütun 0
    const double ph1 = P(i, 1);  // sütun 1
    K[i * 2 + 0] = ph0 * Si00 + ph1 * Si10;
    K[i * 2 + 1] = ph0 * Si01 + ph1 * Si11;
  }

  // x = x + K y
  for (int i = 0; i < 4; ++i) {
    x_[i] += K[i * 2 + 0] * y0 + K[i * 2 + 1] * y1;
  }

  // P = (I - K H) P.  K H yalnızca ilk iki sütunu etkiler.
  std::array<double, 16> Pnew = P_;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      // (K H P)_{ij} = K_{i,0} P_{0,j} + K_{i,1} P_{1,j}
      const double khp = K[i * 2 + 0] * P(0, j) + K[i * 2 + 1] * P(1, j);
      Pnew[i * 4 + j] = P_[i * 4 + j] - khp;
    }
  }
  P_ = Pnew;
}

// ----------------------------------------------------------------------------
// ObstacleTracker
// ----------------------------------------------------------------------------
ObstacleTracker::ObstacleTracker()
: ObstacleTracker(Params{})
{
}

ObstacleTracker::ObstacleTracker(const Params & params)
: params_(params)
{
}

void ObstacleTracker::update(const std::vector<std::pair<double, double>> & detections,
                             double dt)
{
  // 1) Tüm mevcut izleri ileri taşı (tahmin)
  for (auto & t : tracks_) {
    t.kf.predict(dt);
  }

  // 2) Gözlemleri en yakın komşu ile izlere ata (greedy)
  std::vector<bool> track_matched(tracks_.size(), false);
  std::vector<bool> det_matched(detections.size(), false);

  const double max_d2 = params_.association_dist * params_.association_dist;

  for (size_t di = 0; di < detections.size(); ++di) {
    int    best_ti = -1;
    double best_d2 = max_d2;
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (track_matched[ti]) {
        continue;
      }
      const double dx = detections[di].first  - tracks_[ti].kf.x();
      const double dy = detections[di].second - tracks_[ti].kf.y();
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_ti = static_cast<int>(ti);
      }
    }
    if (best_ti >= 0) {
      auto & t = tracks_[static_cast<size_t>(best_ti)];
      t.kf.update(detections[di].first, detections[di].second);
      t.hits++;
      t.misses = 0;
      track_matched[static_cast<size_t>(best_ti)] = true;
      det_matched[di] = true;
    }
  }

  // 3) Eşleşmeyen izler: kaçırıldı say
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (!track_matched[ti]) {
      tracks_[ti].misses++;
    }
  }

  // 4) Eşleşmeyen gözlemler: yeni iz aç
  for (size_t di = 0; di < detections.size(); ++di) {
    if (!det_matched[di]) {
      Track t{next_id_++, KalmanFilter2D(params_.process_noise, params_.measurement_noise), 1, 0};
      t.kf.init(detections[di].first, detections[di].second);
      tracks_.push_back(std::move(t));
    }
  }

  // 5) Çok uzun süredir görülmeyen izleri sil
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
                   [this](const Track & t) { return t.misses > params_.max_misses; }),
    tracks_.end());
}

std::vector<TrackedObstacle> ObstacleTracker::confirmedTracks() const
{
  std::vector<TrackedObstacle> out;
  for (const auto & t : tracks_) {
    if (t.hits >= params_.min_hits_confirmed) {
      out.push_back({t.id, t.kf.x(), t.kf.y(), t.kf.vx(), t.kf.vy(), t.hits, t.misses});
    }
  }
  return out;
}

std::vector<TrackedObstacle> ObstacleTracker::allTracks() const
{
  std::vector<TrackedObstacle> out;
  for (const auto & t : tracks_) {
    out.push_back({t.id, t.kf.x(), t.kf.y(), t.kf.vx(), t.kf.vy(), t.hits, t.misses});
  }
  return out;
}

}  // namespace ika_navigation
