#include <lla2local.h>

using std::placeholders::_1;
using namespace std;

/*
from math import radians, cos, sqrt, atan2, degrees

def flat_earth_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth's radius in meters
    phi1, phi2 = radians(lat1), radians(lat2)
    lambda1, lambda2 = radians(lon1), radians(lon2)
    
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1
    phi_avg = (phi1 + phi2) / 2
    
    # Cartesian differences
    x = R * dlambda * cos(phi_avg)
    y = R * dphi
    
    # Distance
    distance = sqrt(x**2 + y**2)
    
    # Bearing (in radians)
    bearing = atan2(y, x)
    
    # Convert bearing to degrees and normalize to 0-360
    bearing_degrees = (degrees(bearing) + 360) % 360
    
    return distance, bearing_degrees

# Example
lat1, lon1 = 32.678132, -96.922486  # Point 1: minivan rear
lat2, lon2 =  32.678109, -96.922439  # Point 2: minivan front

distance, bearing = flat_earth_distance_and_bearing(lat1, lon1, lat2, lon2)
print(f"Flat-Earth Distance: {distance:.2f} meters")
print(f"Bearing: {bearing:.2f} degrees")
 
North is zero degrees. The sample points are the front and back of my minivan. seems to work, but no warranties expressed or implied 
*/
Lla2Local::Lla2Local() : Node("Lla2Local")
{
  // get the declared origin parameters (in degrees) & save as radians
  this->declare_parameter("origin_lat", 32.779167);  // Dallas
  this->declare_parameter("origin_lon", -96.808891);
  double originLatDeg = this->get_parameter("origin_lat").as_double();
  double originLonDeg = this->get_parameter("origin_lon").as_double();
  originLat_ = originLatDeg * (M_PI / 180.0);
  originLon_ = originLonDeg * (M_PI / 180.0);

  localXyPub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("local_east_north", 10);
  auto qos = rclcpp::SensorDataQoS();
  gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>
    ("fix", qos, std::bind(&Lla2Local::GPSCallback, this, _1 ));
  gpsMonPub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("lla_mon", 10);

  RCLCPP_INFO(this->get_logger(), "lla2local is running");
  RCLCPP_INFO(this->get_logger(), "using origin lat %f lon %f", originLatDeg, originLonDeg);
}

void Lla2Local::GPSCallback(const sensor_msgs::msg::NavSatFix& msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Got data from GNSS (lat, lon): %f, %f"
    , msg.latitude, msg.longitude);

  gpsMonPub_->publish(msg);   // publish a copy of NavSatFix for monitoring/bagging

  // convert measured lat/lon to radians
  roverLat_ = msg.latitude * (M_PI / 180.0);
  roverLon_ = msg.latitude * (M_PI / 180.0);

  // calculate X-Y relative to local origin
  double deltaLat = roverLat_ - originLat_;
  double avgLat = (roverLat_ + originLat_) / 2.0;
  double deltaLon = roverLon_ - originLon_;
  double x = R_ * deltaLon * cos(avgLat);  // distance east of the origin
  double y = R_ * deltaLat;  // distance north of the origin

  // publish X-Y in a PoseStamped msg
  geometry_msgs::msg::PoseStamped local_pose;

  local_pose.header = msg.header;
  local_pose.pose.position.x = x;
  local_pose.pose.position.y = y;
  local_pose.pose.position.z = 0.0;

  RCLCPP_DEBUG(this->get_logger(), "Converted to (x, y): %f %f"
    , local_pose.pose.position.x, local_pose.pose.position.y);

  localXyPub_->publish(local_pose);

}