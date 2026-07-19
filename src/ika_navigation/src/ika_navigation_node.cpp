#include "ika_navigation/ika_navigation_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ika_navigation
{

namespace
{
// Quaternion -> yaw (z ekseni etrafında dönüş)
double quatToYaw(double qx, double qy, double qz, double qw)
{
  const double siny = 2.0 * (qw * qz + qx * qy);
  const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny, cosy);
}
}  // namespace

IkaNavigationNode::IkaNavigationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ika_navigation", options)
{
  // --- Genel parametreler ---
  max_linear_speed_   = declare_parameter<double>("max_linear_speed", 0.6);
  max_angular_speed_  = declare_parameter<double>("max_angular_speed", 1.2);
  slow_down_dist_     = declare_parameter<double>("slow_down_dist", 1.5);
  stop_dist_          = declare_parameter<double>("stop_dist", 0.5);
  scan_angle_offset_  = declare_parameter<double>("scan_angle_offset", 0.0);
  cluster_threshold_  = declare_parameter<double>("cluster_threshold", 0.3);
  angular_gain_       = declare_parameter<double>("angular_gain", 1.5);
  odom_frame_         = declare_parameter<std::string>("odom_frame", "odom");

  // --- Stanley parametreleri ---
  StanleyController::Params sp;
  sp.k          = declare_parameter<double>("stanley.k", 1.5);
  sp.k_soft     = declare_parameter<double>("stanley.k_soft", 0.5);
  sp.max_steer  = declare_parameter<double>("stanley.max_steer", 1.0);
  sp.wheel_base = declare_parameter<double>("stanley.wheel_base", 0.5);
  stanley_ = StanleyController(sp);
  stanley_.setGoalTolerance(declare_parameter<double>("stanley.goal_tolerance", 0.3));

  // --- VFH+ parametreleri ---
  VFHPlus::Params vp;
  vp.num_sectors     = static_cast<int>(declare_parameter<int>("vfh.num_sectors", 72));
  vp.robot_radius    = declare_parameter<double>("vfh.robot_radius", 0.35);
  vp.safety_distance = declare_parameter<double>("vfh.safety_distance", 0.25);
  vp.max_range       = declare_parameter<double>("vfh.max_range", 4.0);
  vp.threshold_low   = declare_parameter<double>("vfh.threshold_low", 3.0);
  vp.threshold_high  = declare_parameter<double>("vfh.threshold_high", 8.0);
  vp.mu1_target      = declare_parameter<double>("vfh.mu1_target", 5.0);
  vp.mu2_current     = declare_parameter<double>("vfh.mu2_current", 2.0);
  vp.mu3_previous    = declare_parameter<double>("vfh.mu3_previous", 2.0);
  vfh_ = VFHPlus(vp);

  // --- Kalman / engel takip parametreleri ---
  ObstacleTracker::Params tp;
  tp.process_noise     = declare_parameter<double>("tracker.process_noise", 1.0);
  tp.measurement_noise = declare_parameter<double>("tracker.measurement_noise", 0.05);
  tp.association_dist  = declare_parameter<double>("tracker.association_dist", 0.6);
  tp.max_misses        = static_cast<int>(declare_parameter<int>("tracker.max_misses", 8));
  tracker_ = ObstacleTracker(tp);

  // --- Referans yol (waypoint'ler): [x0,y0, x1,y1, ...] düz liste ---
  const auto wp_flat = declare_parameter<std::vector<double>>(
    "waypoints", std::vector<double>{});
  std::vector<Waypoint> path;
  for (size_t i = 0; i + 1 < wp_flat.size(); i += 2) {
    path.push_back({wp_flat[i], wp_flat[i + 1]});
  }
  if (path.size() >= 2) {
    stanley_.setPath(path);
    RCLCPP_INFO(get_logger(), "%zu noktalı referans yol yüklendi", path.size());
  } else {
    RCLCPP_WARN(get_logger(),
                "Yol tanımlanmadı ('waypoints' boş); yalnızca engelden kaçınma çalışır");
  }

  // --- ROS arayüzleri ---
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&IkaNavigationNode::onScan, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(10),
    std::bind(&IkaNavigationNode::onOdom, this, std::placeholders::_1));

  cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "tracked_obstacles", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
              "İKA navigasyon düğümü hazır (Stanley + VFH+ + Kalman)");
}

void IkaNavigationNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  pose_.x   = msg->pose.pose.position.x;
  pose_.y   = msg->pose.pose.position.y;
  pose_.yaw = quatToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // İleri hız (gövde çerçevesindeki x bileşeni)
  velocity_  = msg->twist.twist.linear.x;
  have_pose_ = true;
}

// ----------------------------------------------------------------------------
// Taramadan engel kümeleri çıkar: ardışık, mesafe farkı küçük ışınları
// gruplayıp merkezini (centroid) engel noktası kabul et; odom'a taşı.
// ----------------------------------------------------------------------------
std::vector<std::pair<double, double>> IkaNavigationNode::extractObstacles(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<std::pair<double, double>> obstacles;
  const double cos_y = std::cos(pose_.yaw);
  const double sin_y = std::sin(pose_.yaw);

  double sum_x = 0.0, sum_y = 0.0;
  int    count = 0;
  double prev_r = std::numeric_limits<double>::infinity();

  auto flush = [&]() {
    if (count > 0) {
      const double cx = sum_x / count;  // robot çerçevesi centroid
      const double cy = sum_y / count;
      // robot -> odom dönüşümü (poz biliniyorsa)
      double ox = cx, oy = cy;
      if (have_pose_) {
        ox = pose_.x + cos_y * cx - sin_y * cy;
        oy = pose_.y + sin_y * cx + cos_y * cy;
      }
      obstacles.emplace_back(ox, oy);
    }
    sum_x = sum_y = 0.0;
    count = 0;
  };

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double d = scan.ranges[i];
    if (!std::isfinite(d) || d <= scan.range_min || d > scan.range_max) {
      flush();
      prev_r = std::numeric_limits<double>::infinity();
      continue;
    }
    // Ardışık ışın mesafe farkı büyükse yeni küme başlat
    if (count > 0 && std::fabs(d - prev_r) > cluster_threshold_) {
      flush();
    }
    const double beta = scan.angle_min + static_cast<double>(i) * scan.angle_increment
                        + scan_angle_offset_;
    sum_x += d * std::cos(beta);
    sum_y += d * std::sin(beta);
    ++count;
    prev_r = d;
  }
  flush();
  return obstacles;
}

// ----------------------------------------------------------------------------
// ANA BORU HATTI: her tarama geldiğinde çalışır.
// ----------------------------------------------------------------------------
void IkaNavigationNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // dt hesabı (Kalman tahmini için)
  const rclcpp::Time stamp = msg->header.stamp;
  double dt = 0.1;
  if (have_last_scan_) {
    dt = (stamp - last_scan_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.1;  // saat sıçraması / ilk mesaj koruması
    }
  }
  last_scan_time_ = stamp;
  have_last_scan_ = true;

  // 1) KALMAN: engelleri odom çerçevesinde takip et (sapmasınlar)
  const auto detections = extractObstacles(*msg);
  tracker_.update(detections, dt);
  const auto tracks = tracker_.confirmedTracks();
  publishMarkers(tracks);

  // 2) STANLEY: yolu takip için istenen yönü bul (robot çerçevesinde)
  double target_angle = 0.0;  // varsayılan: düz ileri
  bool   goal_reached = false;
  if (stanley_.hasPath() && have_pose_) {
    const auto st = stanley_.computeSteering(pose_, velocity_);
    if (st.valid) {
      target_angle = st.steering;   // araç çerçevesinde düzeltme yönü
      goal_reached = st.goal_reached;
    }
  }

  // 3) VFH+: hedefe en yakın GÜVENLİ yönü seç
  const auto vr = vfh_.computeSteering(msg->ranges, msg->angle_min,
                                       msg->angle_increment, target_angle);

  // 4) Komut üret
  geometry_msgs::msg::Twist cmd;

  if (goal_reached) {
    // Hedefe varıldı: dur
    cmd_pub_->publish(cmd);  // sıfır Twist
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Hedefe ulaşıldı, duruluyor");
    return;
  }

  if (vr.blocked) {
    // Önü tamamen kapalı: dur (isteğe göre yerinde dönüş eklenebilir)
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0.0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Yol kapalı — duruluyor (en yakın engel %.2f m)", vr.nearest_obstacle);
    cmd_pub_->publish(cmd);
    return;
  }

  // Açısal hız: seçilen yöne orantısal, sınırlı
  double ang = angular_gain_ * vr.steering_angle;
  ang = std::clamp(ang, -max_angular_speed_, max_angular_speed_);

  // Doğrusal hız: engel yakınlığına ve dönüş keskinliğine göre ölçekle
  double lin = max_linear_speed_;
  const double nearest = vr.nearest_obstacle;
  if (std::isfinite(nearest)) {
    if (nearest < stop_dist_) {
      lin = 0.0;
    } else if (nearest < slow_down_dist_) {
      lin *= (nearest - stop_dist_) / (slow_down_dist_ - stop_dist_);
    }
  }
  // Keskin dönüşte yavaşla (yönelim hatası büyükse önce dön)
  lin *= std::max(0.2, std::cos(vr.steering_angle));

  cmd.linear.x  = lin;
  cmd.angular.z = ang;
  cmd_pub_->publish(cmd);
}

void IkaNavigationNode::publishMarkers(const std::vector<TrackedObstacle> & tracks)
{
  visualization_msgs::msg::MarkerArray arr;

  // Önce eski işaretçileri temizle
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = odom_frame_;
  clear.header.stamp    = now();
  clear.action          = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  for (const auto & t : tracks) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = odom_frame_;
    m.header.stamp    = now();
    m.ns              = "obstacles";
    m.id              = t.id;
    m.type            = visualization_msgs::msg::Marker::CYLINDER;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = t.x;
    m.pose.position.y = t.y;
    m.pose.position.z = 0.25;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = 0.3;
    m.scale.z = 0.5;
    // Hareketli engel (kayar bariyer) kırmızı, sabit engel yeşil tonlansın
    const double speed = std::hypot(t.vx, t.vy);
    m.color.r = speed > 0.15 ? 1.0f : 0.1f;
    m.color.g = speed > 0.15 ? 0.1f : 1.0f;
    m.color.b = 0.1f;
    m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    arr.markers.push_back(m);
  }
  marker_pub_->publish(arr);
}

}  // namespace ika_navigation

// ============================================================================
// GİRİŞ NOKTASI
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ika_navigation::IkaNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
