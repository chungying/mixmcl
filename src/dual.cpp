// Signal handling
#include <signal.h>
// roscpp
#include "ros/ros.h"
#include <mixmcl/MixmclNode.h>
#include <sstream>
using namespace std;
using namespace nuklei;

class DualNode;
boost::shared_ptr<DualNode> dual_node_ptr;

void sigintHandler(int sig)
{
  ros::shutdown();
}

class DualNode : public MixmclNode
{
  public:
    DualNode();
    ~DualNode(){};
  protected:
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void RCCB();
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual");
  ros::NodeHandle nh;
  signal(SIGINT, sigintHandler);
  dual_node_ptr.reset(new DualNode());
  ros::spin();
  dual_node_ptr.reset();
  return(0);
}

DualNode::DualNode() : MixmclNode()
{

  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
  if(laser_scan_filter_!=NULL)
    delete laser_scan_filter_;
  laser_scan_filter_ = 
    new tf::MessageFilter<sensor_msgs::LaserScan>(
          *laser_scan_sub_, 
          *tf_, 
          odom_frame_id_, 
          100);
  laser_scan_filter_->registerCallback(
    boost::bind(
      &DualNode::laserReceived,
      this, _1));
}

void DualNode::RCCB()
{
  ROS_INFO("DualNode::RCCB() is called.");
  delete laser_scan_filter_;
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&DualNode::laserReceived,
                                                   this, _1));
  
}

void DualNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new amcl::AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();
    tf::Stamped<tf::Pose> ident (
      tf::Transform(
        tf::createIdentityQuaternion(),
        tf::Vector3(0,0,0)),
      ros::Time(),
      laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }
    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  }
  else
  {
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }
  amcl::AMCLLaserData ldata;
  ldata.sensor = lasers_[laser_index];
  ldata.range_count = laser_scan->ranges.size();

  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                    laser_scan->header.frame_id);
  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                    laser_scan->header.frame_id);
  try
  {
    tf_->transformQuaternion(base_frame_id_, min_q, min_q);
    tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
             e.what());
    return;
  }

  double angle_min = tf::getYaw(min_q);
  double angle_increment = tf::getYaw(inc_q) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

  //ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);

  // Apply range min/max thresholds, if the user supplied them
  if(laser_max_range_ > 0.0)
    ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
  else
    ldata.range_max = laser_scan->range_max;
  double range_min;
  if(laser_min_range_ > 0.0)
    range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
  else
    range_min = laser_scan->range_min;
  // The AMCLLaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];
  ROS_ASSERT(ldata.ranges);
  for(int i=0;i<ldata.range_count;i++)
  {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if(laser_scan->ranges[i] <= range_min)
      ldata.ranges[i][0] = ldata.range_max;
    else
      ldata.ranges[i][0] = laser_scan->ranges[i];
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
            (i * angle_increment);
  }


  //convert ldata to features, x, y, and dist.
  laser_feature_t feature = polygonCentroid(ldata);
  std::stringstream ss;
  boost::shared_ptr<const KernelCollection> tree = kcgrid_->getTree(feature.x, feature.y, feature.dist, ss);
  ROS_DEBUG_STREAM(ss);
  KernelCollection::const_iterator iter = tree->begin();
  KernelCollection::const_iterator end = tree->end();
  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(tree->size());
  for(int i = 0;iter != end ; ++iter, ++i)
  {
    kernel::se3 se3_pose(*iter);
    pf_vector_t vec_pose;
    se3ToPose(se3_pose, vec_pose); 
    tf::poseTFToMsg(
      tf::Pose(
        tf::createQuaternionFromYaw(
          vec_pose.v[2]),
        tf::Vector3(
          vec_pose.v[0],
          vec_pose.v[1], 
          0)),
      cloud_msg.poses[i]);
  }
  particlecloud2_pub_.publish(cloud_msg);
  ROS_INFO("%lu particles correspond to feature x:%lf, y:%lf, d:%lf", tree->size(), feature.x, feature.y, feature.dist);

}
