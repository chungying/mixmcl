#include <boost/shared_ptr.hpp>

// Signal handling
#include <signal.h>

// roscpp
#include "ros/ros.h"

#include "mixmcl/SamplingNode.h"
boost::shared_ptr<SamplingNode> node_ptr;

void sigintHandler(int sig)
{
  // Flush memory into disk when we're shutting down.
  ros::shutdown();
}


int main(int argc, char** argv)
{
  try{
  ros::init(argc, argv, "sampling");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string valfullpath, valtimestamp;
  std::string nodename = ros::this_node::getName();
  ros::Rate wait(1);
  std::string keyfullpath = nodename + fp;
  while(!nh.getParam(keyfullpath, valfullpath))
  {
    ROS_INFO("waiting 1 second for parameter %s/%s", nodename.c_str(), fp.c_str());
    wait.sleep();
  }
  ROS_INFO("%s: %s", keyfullpath.c_str(), valfullpath.c_str());
  private_nh.setParam(keyfullpath, valfullpath);

  std::string keytimestamp = nodename + ts;
  while(!nh.getParam(keytimestamp, valtimestamp))
  {
    ROS_INFO("waiting 1 second for parameter %s/%s",nodename.c_str(), ts.c_str());
    wait.sleep();
  }
  ROS_INFO("%s: %s", keytimestamp.c_str(), valtimestamp.c_str());
  private_nh.setParam(keytimestamp, valtimestamp);
  int slowRatio;
  double ip_x, ip_y, ip_a;
  private_nh.param("slowRatio", slowRatio, int(50));
/*  private_nh.param("ip_x", ip_x, double(0));
  private_nh.param("ip_y", ip_y, double(0));
  private_nh.param("ip_a", ip_a, double(0));*/
  ROS_INFO("slowRatio: %d", slowRatio);
  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  node_ptr.reset(new SamplingNode());
  node_ptr->moveRobotUniformly();
  ROS_INFO("start collecting sampling information");
  int hz = 50;
  ros::Rate r(hz);
  int count = 0;
  while(nh.ok())
  {
    count++;
    ros::spinOnce();
    if(count%slowRatio == 0)
      node_ptr->moveRobotUniformly();
    r.sleep();
  }
  ROS_INFO("SamplingNode ends.");
  node_ptr.reset();
  }
  catch(ros::InvalidNameException& e)
  {
    ROS_INFO("%s",e.what());
  }
  return(0);

}


