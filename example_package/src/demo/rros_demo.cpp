#include <ros/ros.h>
#include <rros_template_repo/MessageRROS.h>

#include <rros.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "rros_demo");
  
  rros_template_repo::MessageRROS msg;
  std::cout << msg.number << std::endl;
  std::cout << msg.size << std::endl;

  ros::spin();
  ros::shutdown();
  return 0;
}
