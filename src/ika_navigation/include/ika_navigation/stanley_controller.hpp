#pragma once

#include <cstddef>
#include <vector>

namespace ika_navigation
{

// 2B poz (dünya/odom çerçevesi)
struct Pose2D
{
  double x;
  double y;
  double yaw;  // rad, +x'ten CCW
};

struct Waypoint
{
  double x;
  double y;
};

// ============================================================================
// STANLEY CONTROLLER — yol takip (path tracking) direksiyon kontrolcüsü
//
// Stanford DARPA aracı "Stanley"den. Ön aks orta noktasını referans alarak
// iki hatayı sıfırlamaya çalışır:
//   1) Yönelim hatası (heading error): aracın yönü ile yolun teğeti farkı
//   2) Çapraz konum hatası (cross-track error, e): ön aksın yola dik uzaklığı
//
// Kontrol yasası:
//   δ = θ_e + atan2(k · e, k_soft + v)
//
//   θ_e     : yönelim hatası
//   e       : çapraz hata (yolun soluna pozitif)
//   v       : aracın ileri hızı
//   k       : kazanç (büyük k -> yola daha agresif dönüş)
//   k_soft  : düşük hızda kararlılık için yumuşatma terimi (v→0'da patlamayı önler)
//
// Tank (paletli / diferansiyel sürüş) için: δ doğrudan bir dönüş açısı
// komutudur; düğüm bunu açısal hıza (ω = v·tan(δ)/L ya da orantısal) çevirir.
// ============================================================================
class StanleyController
{
public:
  struct Params
  {
    double k          = 1.5;   // çapraz hata kazancı
    double k_soft     = 0.5;   // yumuşatma (m/s), düşük hız kararlılığı
    double max_steer  = 1.0;   // direksiyon açısı sınırı (rad, ~57°)
    double wheel_base = 0.5;   // ön-arka aks mesafesi (tankta sanal L)
    bool   loop_path  = false; // yol döngüsel mi (son noktadan başa dön)
  };

  struct Result
  {
    bool   valid;          // takip edilecek geçerli yol var mı?
    double steering;       // direksiyon açısı (rad), [-max_steer, max_steer]
    double cross_track;    // anlık çapraz hata (m) — telemetri/log için
    double heading_error;  // anlık yönelim hatası (rad)
    size_t target_index;   // yolda hedeflenen en yakın segment indeksi
    bool   goal_reached;   // yolun sonuna ulaşıldı mı?
  };

  StanleyController();
  explicit StanleyController(const Params & params);

  void setPath(const std::vector<Waypoint> & path);
  bool hasPath() const { return path_.size() >= 2; }

  // Verilen poz ve ileri hız için direksiyon komutunu hesapla.
  Result computeSteering(const Pose2D & pose, double velocity);

  // Hedefe varış toleransı (m): son waypoint bu mesafeye girince goal_reached
  void setGoalTolerance(double tol) { goal_tolerance_ = tol; }

private:
  Params                params_;
  std::vector<Waypoint> path_;
  double                goal_tolerance_{0.3};

  // Ön aks noktasına en yakın yol segmentini bulur; segment üzerine izdüşüm
  // ve çapraz hatayı döndürür.
  size_t nearestSegment(double fx, double fy, double & cross_track,
                        double & seg_heading) const;
};

}  // namespace ika_navigation
