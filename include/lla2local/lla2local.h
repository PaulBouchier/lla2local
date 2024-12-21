#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>

class Lla2Local : public rclcpp::Node
{ 
  public:
    Lla2Local();

  private:
    void GPSCallback(const sensor_msgs::msg::NavSatFix & msgs);

    const double R_ = 6371000.0;  // Earth's radius in meters
    double originLat_;          // local origin latitude (radians)
    double originLon_;          // local origin longitude (radians)
    double roverLat_;           // measured latitude (radians)
    double roverLon_;           // measured longitude (radians)

    // publishes X-Y relative to the local origin
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localXyPub_;
    // subscribes to GNSS fix
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    // republishes GNSS fix with QOS that can be subscribed to by e.g. rqt_topic
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsMonPub_;
};

