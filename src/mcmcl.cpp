#include <boost/shared_ptr.hpp>

// Signal handling
#include <signal.h>

// roscpp
#include "ros/ros.h"

#include "mcmcl/McmclNode.h"

boost::shared_ptr<McmclNode> mcmcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  mcmcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "mcmcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  mcmcl_node_ptr.reset(new McmclNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }
  else if ((argc == 3) && (std::string(argv[1]) == "--run-from-bag"))
  {
    mcmcl_node_ptr->runFromBag(argv[2]);
  }

  // Without this, our boost locks are not shut down nicely
  mcmcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}
