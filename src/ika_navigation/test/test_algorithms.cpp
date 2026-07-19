// Stanley / VFH+ / Kalman algoritmaları için temel birim testleri.
//   colcon test --packages-select ika_navigation
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "ika_navigation/stanley_controller.hpp"
#include "ika_navigation/vfh_plus.hpp"
#include "ika_navigation/kalman_filter.hpp"

using namespace ika_navigation;

// ---------------------------------------------------------------------------
// KALMAN: sabit hızla giden bir hedefi yakınsayarak takip etmeli
// ---------------------------------------------------------------------------
TEST(KalmanFilter, ConvergesToConstantVelocity)
{
  KalmanFilter2D kf(1.0, 0.05);
  kf.init(0.0, 0.0);

  const double dt = 0.1;
  const double vx = 1.0;  // gerçek hız
  double true_x = 0.0;

  for (int i = 0; i < 50; ++i) {
    true_x += vx * dt;
    kf.predict(dt);
    // Küçük gürültülü ölçüm (deterministik seri)
    const double noise = 0.01 * std::sin(i * 1.3);
    kf.update(true_x + noise, 0.0);
  }

  EXPECT_NEAR(kf.x(), true_x, 0.1);
  EXPECT_NEAR(kf.vx(), vx, 0.2);   // hız kestirimi gerçek hıza yakınsamalı
}

// ---------------------------------------------------------------------------
// ObstacleTracker: aynı fiziksel engel aynı id ile stabil kalmalı
// ---------------------------------------------------------------------------
TEST(ObstacleTracker, StableIdAcrossFrames)
{
  ObstacleTracker::Params p;
  p.min_hits_confirmed = 2;
  ObstacleTracker tracker(p);

  for (int i = 0; i < 5; ++i) {
    // Hafif gürültülü ama sabit bir engel
    std::vector<std::pair<double, double>> dets = {{2.0 + 0.01 * i, 1.0}};
    tracker.update(dets, 0.1);
  }

  auto tracks = tracker.confirmedTracks();
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NEAR(tracks[0].x, 2.0, 0.2);
  EXPECT_NEAR(tracks[0].y, 1.0, 0.1);
}

// ---------------------------------------------------------------------------
// VFH+: önü açık, sağ tarafı kapalıyken sola/ileri yön seçmeli
// ---------------------------------------------------------------------------
TEST(VFHPlus, AvoidsBlockedSector)
{
  VFHPlus::Params p;
  p.num_sectors = 72;
  p.max_range   = 4.0;
  VFHPlus vfh(p);

  // 360 ışın, 1° aralık. Hepsi açık (inf) başlasın.
  const int n = 360;
  std::vector<float> ranges(n, std::numeric_limits<float>::infinity());
  // Sağ tarafta (-90° civarı) yakın engel duvarı koy
  for (int deg = -110; deg <= -70; ++deg) {
    int idx = (deg + 360) % 360;
    ranges[idx] = 0.6f;
  }

  const double angle_min = -M_PI;
  const double inc = 2.0 * M_PI / n;
  // Hedef düz ileri (0 rad)
  auto r = vfh.computeSteering(ranges, angle_min, inc, 0.0);

  EXPECT_TRUE(r.valid);
  EXPECT_FALSE(r.blocked);
  // Seçilen yön sağdaki engelden uzak olmalı (sağ = negatif açı)
  EXPECT_GT(r.steering_angle, -M_PI_2 + 0.3);
}

// ---------------------------------------------------------------------------
// VFH+: her yön kapalıysa blocked=true dönmeli
// ---------------------------------------------------------------------------
TEST(VFHPlus, DetectsFullyBlocked)
{
  VFHPlus::Params p;
  p.num_sectors = 72;
  p.max_range   = 4.0;
  VFHPlus vfh(p);

  const int n = 360;
  std::vector<float> ranges(n, 0.4f);  // her yön çok yakın engel

  auto r = vfh.computeSteering(ranges, -M_PI, 2.0 * M_PI / n, 0.0);
  EXPECT_TRUE(r.blocked);
}

// ---------------------------------------------------------------------------
// STANLEY: yolun soluna sapmış araç sağa (negatif) direksiyon vermeli
// ---------------------------------------------------------------------------
TEST(StanleyController, CorrectsCrossTrackError)
{
  StanleyController::Params p;
  p.wheel_base = 0.0;  // testte ön aks = poz
  StanleyController ctrl(p);

  // +x yönünde düz bir yol
  std::vector<Waypoint> path = {{0.0, 0.0}, {10.0, 0.0}};
  ctrl.setPath(path);

  // Araç yol yönünde bakıyor ama +y (sol) tarafa 1 m kaymış
  Pose2D pose{2.0, 1.0, 0.0};
  auto r = ctrl.computeSteering(pose, 1.0);

  ASSERT_TRUE(r.valid);
  EXPECT_LT(r.steering, 0.0);           // sağa dönerek yola geri gelmeli
  EXPECT_NEAR(r.cross_track, 1.0, 0.05); // sol tarafa pozitif çapraz hata
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
