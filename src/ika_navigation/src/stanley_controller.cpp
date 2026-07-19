#include "ika_navigation/stanley_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ika_navigation
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double wrapPi(double a)
{
  while (a >= M_PI)  { a -= kTwoPi; }
  while (a < -M_PI)  { a += kTwoPi; }
  return a;
}
}  // namespace

StanleyController::StanleyController()
: StanleyController(Params{})
{
}

StanleyController::StanleyController(const Params & params)
: params_(params)
{
}

void StanleyController::setPath(const std::vector<Waypoint> & path)
{
  path_ = path;
}

// Ön aksa en yakın segmenti bul, izdüşümü hesapla.
// cross_track: yolun soluna pozitif (işaretli), seg_heading: segment teğet açısı.
size_t StanleyController::nearestSegment(double fx, double fy,
                                         double & cross_track,
                                         double & seg_heading) const
{
  size_t best_i   = 0;
  double best_d2  = std::numeric_limits<double>::infinity();
  double best_ct  = 0.0;
  double best_hdg = 0.0;

  const size_t n_seg = path_.size() - 1;
  for (size_t i = 0; i < n_seg; ++i) {
    const double ax = path_[i].x,     ay = path_[i].y;
    const double bx = path_[i + 1].x, by = path_[i + 1].y;
    const double dx = bx - ax,        dy = by - ay;
    const double seg_len2 = dx * dx + dy * dy;
    if (seg_len2 < 1e-9) {
      continue;  // yozlaşmış segment
    }

    // Ön aks noktasının segment üzerine izdüşüm parametresi t ∈ [0,1]
    double t = ((fx - ax) * dx + (fy - ay) * dy) / seg_len2;
    t = std::clamp(t, 0.0, 1.0);

    const double px = ax + t * dx;   // en yakın nokta
    const double py = ay + t * dy;
    const double ex = fx - px;
    const double ey = fy - py;
    const double d2 = ex * ex + ey * ey;

    if (d2 < best_d2) {
      best_d2  = d2;
      best_i   = i;
      best_hdg = std::atan2(dy, dx);
      // İşaretli çapraz hata: segment yönü ile (araç->nokta) vektörünün
      // çapraz çarpımının işareti. Sol taraf pozitif.
      const double cross = dx * ey - dy * ex;
      const double dist  = std::sqrt(d2);
      best_ct = (cross >= 0.0) ? dist : -dist;
    }
  }

  cross_track = best_ct;
  seg_heading = best_hdg;
  return best_i;
}

StanleyController::Result StanleyController::computeSteering(const Pose2D & pose,
                                                            double velocity)
{
  Result r{};
  r.valid = false;

  if (!hasPath()) {
    return r;
  }

  // Ön aks orta noktası: arka referanslı poz + wheel_base ileride
  const double fx = pose.x + params_.wheel_base * std::cos(pose.yaw);
  const double fy = pose.y + params_.wheel_base * std::sin(pose.yaw);

  // Hedefe (son waypoint) varış kontrolü
  const auto & goal = path_.back();
  const double gdx  = goal.x - fx;
  const double gdy  = goal.y - fy;
  if (std::sqrt(gdx * gdx + gdy * gdy) < goal_tolerance_) {
    r.valid        = true;
    r.steering     = 0.0;
    r.goal_reached = true;
    r.target_index = path_.size() - 1;
    return r;
  }

  double cross_track = 0.0;
  double seg_heading = 0.0;
  const size_t seg = nearestSegment(fx, fy, cross_track, seg_heading);

  // 1) Yönelim hatası: yol teğeti - araç yönü
  const double heading_error = wrapPi(seg_heading - pose.yaw);

  // 2) Çapraz hata terimi: atan2(-k·e, k_soft + v)
  //    e sola pozitif tanımlı; araç solda ise sağa (negatif) dönmeli, bu yüzden
  //    -e kullanılır. velocity geri viteste <0 olabilir -> mutlak değer.
  const double v = std::fabs(velocity);
  const double cross_term = std::atan2(-params_.k * cross_track,
                                       params_.k_soft + v);

  double steering = heading_error + cross_term;
  steering = std::clamp(steering, -params_.max_steer, params_.max_steer);

  r.valid         = true;
  r.steering      = steering;
  r.cross_track   = cross_track;
  r.heading_error = heading_error;
  r.target_index  = seg;
  r.goal_reached  = false;
  return r;
}

}  // namespace ika_navigation
