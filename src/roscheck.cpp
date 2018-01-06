#include "ros/ros.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "check_ros_master");
  if(argc == 2)
    std::cout << "argv[1] is " << argv[1] << std::endl;
  if(ros::master::check())
    return 0;
  else
    return 1;
}
