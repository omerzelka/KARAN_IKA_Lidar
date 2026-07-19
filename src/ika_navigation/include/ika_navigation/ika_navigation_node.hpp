#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ika_navigation/stanley_controller.hpp"
#include "ika_navigation/vfh_plus.hpp"
#include "ika_navigation/kalman_filter.hpp"

namespace ika_navigation
{

// ============================================================================
// İKA NAVİGASYON DÜĞÜMÜ
//
// Üç algoritmayı tek boru hattında birleştirir:
//
//   /odom ──► STANLEY ──► hedef yön (yolu takip et)
//                │
//   /scan ──► VFH+ ◄──────┘  (hedefe en yakın GÜVENLİ yönü seç)
//     │          │
//     └──► KALMAN (engelleri odom çerçevesinde takip et, sapmayı önle)
//                │
//                ▼
//            /cmd_vel  (geometry_msgs/Twist)
//
// Stanley "nereye gitmek istediğimizi", VFH+ "oraya çarpmadan nasıl
// gidileceğini", Kalman ise "engellerin gerçek/kararlı konumunu" verir.
// ============================================================================
class IkaNavigationNode : public rclcpp::Node
{
public:
  explicit IkaNavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Taramadan engel kümelerini çıkar (robot çerçevesi -> odom çerçevesi)
  std::vector<std::pair<double, double>> extractObstacles(
    const sensor_msgs::msg::LaserScan & scan) const;

  // Takip edilen engelleri RViz için MarkerArray olarak yayınla
  void publishMarkers(const std::vector<TrackedObstacle> & tracks);

  // --- Algoritmalar ---
  StanleyController stanley_;
  VFHPlus           vfh_;
  ObstacleTracker   tracker_;

  // --- ROS arayüzleri ---
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // --- Durum ---
  Pose2D       pose_{0.0, 0.0, 0.0};
  double       velocity_{0.0};
  bool         have_pose_{false};
  rclcpp::Time last_scan_time_;
  bool         have_last_scan_{false};

  // --- Parametreler ---
  double max_linear_speed_{0.6};   // m/s
  double max_angular_speed_{1.2};  // rad/s
  double slow_down_dist_{1.5};     // engel bu mesafede hız düşürülür
  double stop_dist_{0.5};          // bu mesafede tam dur
  double scan_angle_offset_{0.0};  // lidar montaj açısı düzeltmesi (rad)
  double cluster_threshold_{0.3};  // engel kümeleme mesafe eşiği (m)
  double angular_gain_{1.5};       // direksiyon açısı -> açısal hız kazancı
  std::string odom_frame_{"odom"};
};

}  // namespace ika_navigation
