#include <lla2local.h>

#include <cstdio>

int main(int argc, char ** argv)
{
  printf("Starting lla2local\n");

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Lla2Local>());

  rclcpp::shutdown();
  return 0;
}