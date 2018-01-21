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
  double freq;
  private_nh.param("frequency", freq, 0.0);
  boost::shared_ptr<ros::Rate> rate;
  if(freq>0)
    rate.reset(new ros::Rate(freq));
  // Override default sigint handler
  signal(SIGINT, sigintHandler);
  ROS_INFO("frequency is %f", freq);
  // Make our node available to sigintHandler
  node_ptr.reset(new SamplingNode());
  ROS_INFO("start collecting sampling information");
  int count = 0;
  while(nh.ok())
  {
    count++;
    ros::spinOnce();
    node_ptr->sampling();
    if(freq>0)
      rate->sleep();
  }

  ROS_INFO("SamplingNode ends.");
  node_ptr.reset();
  }
  catch(ros::InvalidNameException& e)
  {
    ROS_INFO("%s",e.what());
  }
  return 0;
}


