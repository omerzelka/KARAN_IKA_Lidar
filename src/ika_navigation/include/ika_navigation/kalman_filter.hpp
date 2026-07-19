#pragma once

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace ika_navigation
{

// ============================================================================
// 2B SABİT HIZ (Constant Velocity) KALMAN FİLTRESİ
//
// Durum vektörü:  x = [px, py, vx, vy]^T   (konum + hız, dünya/odom çerçevesi)
// Ölçüm vektörü:  z = [px, py]^T           (yalnızca konum gözlenir)
//
// Amaç: Tank hareket ederken lidardan gelen gürültülü engel konumlarını
// zamanla düzgünleştirmek. Ham lidar ölçümü tur-be-tur birkaç cm sıçrar;
// filtresiz bakınca sabit bir koni "sapıyormuş" gibi görünür. Sabit hız
// modeli bu gürültüyü bastırır ve engelin gerçek konumunu/hızını kestirir.
//
// Denklemler (standart lineer Kalman):
//   Tahmin:   x' = F x            P' = F P F^T + Q
//   Güncelle: y  = z - H x'       S = H P' H^T + R
//             K  = P' H^T S^-1     x = x' + K y     P = (I - K H) P'
// ============================================================================
class KalmanFilter2D
{
public:
  // process_noise  (q): modelin ne kadar "serbest" olduğu (ivme belirsizliği)
  // measurement_noise (r): lidar konum ölçümünün standart sapması ~ varyansı
  KalmanFilter2D(double process_noise = 1.0, double measurement_noise = 0.05);

  // İlk ölçümle durumu başlat (hız = 0 varsayılır)
  void init(double px, double py);

  // dt saniye ilerlet (yeni ölçüm gelmeden önce çağrılır)
  void predict(double dt);

  // Konum ölçümüyle güncelle
  void update(double px, double py);

  double x() const { return x_[0]; }
  double y() const { return x_[1]; }
  double vx() const { return x_[2]; }
  double vy() const { return x_[3]; }

  bool initialized() const { return initialized_; }

  // Kestirim belirsizliğinin izi (iz büyükse takip zayıf; kaybı tespit için)
  double positionUncertainty() const { return P_[0] + P_[5]; }  // Pxx + Pyy

private:
  std::array<double, 4>  x_{};    // durum
  std::array<double, 16> P_{};    // kovaryans (4x4, satır-öncelikli)
  double q_;                      // süreç gürültüsü ölçeği
  double r_;                      // ölçüm gürültüsü varyansı
  bool   initialized_{false};
};

// ============================================================================
// ÇOKLU ENGEL TAKİPÇİSİ (Multi-Object Tracker)
//
// Her tespit edilen engele (koni, kayar bariyer...) bir KalmanFilter2D atar.
// Yeni gözlemleri en yakın komşu (nearest-neighbor) ile mevcut izlere eşler;
// eşleşmeyen gözlemler yeni iz açar, uzun süre görülmeyen izler silinir.
//
// Bu sayede tank ilerlerken aynı fiziksel engel aynı "track_id" ile stabil
// kalır — konumu sürekli filtrelenir, gürültüyle sapmaz.
// ============================================================================
struct TrackedObstacle
{
  int    id;
  double x;      // filtrelenmiş konum (odom çerçevesi)
  double y;
  double vx;     // kestirilen hız (kayar engel tespiti için)
  double vy;
  int    hits;   // kaç kez gözlemlendi (olgunluk ölçüsü)
  int    misses; // arka arkaya kaç kez görülmedi
};

class ObstacleTracker
{
public:
  struct Params
  {
    double process_noise      = 1.0;   // KF süreç gürültüsü
    double measurement_noise  = 0.05;  // KF ölçüm gürültüsü (m^2)
    double association_dist    = 0.6;   // gözlem-iz eşleştirme yarıçapı (m)
    int    max_misses          = 8;     // bu kadar kaçırılınca iz silinir
    int    min_hits_confirmed  = 3;     // "onaylı" iz için gereken gözlem
  };

  ObstacleTracker();
  explicit ObstacleTracker(const Params & params);

  // Bir tarama çevrimindeki engel konumlarını (odom çerçevesinde) işler.
  // dt: son çağrıdan bu yana geçen süre.
  void update(const std::vector<std::pair<double, double>> & detections, double dt);

  // Yalnızca onaylanmış (kararlı) izleri döndürür
  std::vector<TrackedObstacle> confirmedTracks() const;

  // Tüm aktif izler (henüz onaylanmamış dahil)
  std::vector<TrackedObstacle> allTracks() const;

private:
  struct Track
  {
    int            id;
    KalmanFilter2D kf;
    int            hits{1};
    int            misses{0};
  };

  Params             params_;
  std::vector<Track> tracks_;
  int                next_id_{0};
};

}  // namespace ika_navigation
