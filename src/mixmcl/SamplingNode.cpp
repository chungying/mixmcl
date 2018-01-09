#include "mixmcl/SamplingNode.h"
#include "mcl/MCL.cpp"
template class MCL<SamplingNode>;

void SamplingNode::raycasting(
    amcl::AMCLLaser* self,
    const pf_vector_t rpose, 
    const int range_count,
    const double range_max,
    const double range_min,
    const double angle_min,
    const double angle_increment,
    amcl::AMCLLaserData& ldata)
{
  // 2.3 laser sensor information
  //angle_min lamin
  //angle_max lamax
  //angle_increment lares
  //range_min lrmin
  //range_max lrmax
  //range_count lrnum
  ldata.range_count = range_count;
  ldata.range_max = range_max;

  // 2.2 retreive occupancy grid map
  // self->map
  ldata.sensor = self;
  // Take account of the laser pose relative to the robot
  pf_vector_t lpose = pf_vector_coord_add(self->laser_pose, rpose);
  // 2.4 ray-casting for each beam
  ldata.ranges = new double[ldata.range_count][2];
  double map_range;
  for (int i = 0; i < ldata.range_count; ++i)
  {
    ldata.ranges[i][1] = angle_min + (i * angle_increment);
    // Compute the range according to the map
    map_range = map_calc_range(
                  self->map, 
                  lpose.v[0], 
                  lpose.v[1],
                  lpose.v[2] + ldata.ranges[i][1], 
                  range_max);
    if(map_range <= range_min || map_range > range_max)
      ldata.ranges[i][0] = range_max;
    else
      ldata.ranges[i][0] = map_range;
  }
  // 2.5 return AMCLLaserData ldata
}

SamplingNode::SamplingNode():
  MCL(),
  laserUpdated(false),
  dataout_ptr_(),
  paramout_ptr_(),
  data_count_(0)
{
   boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  //one text file for storing parameters and data file name.
  /*data_count_ sampling number 
  **noise laser gaussian noise std dev
  **lrmin laser_min_range_
  **lrmax laser_max_range_
  **lares laser angle resolution
  **lamin laser minimum angle 
  **lamax laser maximum angle 
  **fxmin minimum feature x
  **fxmax maximum feature x
  **fymin minimum feature y
  **fymax maximum feature y
  **fdmin minimum feature dist
  **fdmax maximum feature dist
  */
  //another text file for storing collecting poses and features
  std::string nodename = ros::this_node::getName();
  std::string keyfullpath = nodename + fp;
  std::string keytimestamp = nodename + ts;
  private_nh_.param(keyfullpath, output_dir_, std::string("./"));
  private_nh_.param(keytimestamp, output_prefix_, std::string("noprefix"));
  output_filename_data_ = output_dir_ + "/" + output_prefix_ + "-data.bin";
  output_filename_param_ = output_dir_ + "/" + output_prefix_ + "-param.txt";
  ROS_INFO("output_filename_data:%s", output_filename_data_.c_str());
  ROS_INFO("output_filename_param:%s",output_filename_param_.c_str());
  //define two output streams.
  dataout_ptr_.reset( new dataio::DataOut(output_filename_data_));
  paramout_ptr_.reset( new paramio::ParamOut(output_filename_param_));
  if(!private_nh_.getParam("laser_noise", noise))
    noise = -1.0;
  private_nh_.param("max_data_count", max_data_count_, int(10000));
  //reset callback function to SamplingNode::laserReceived(...)
  ROS_INFO("reset laserReceived callback function");
  laser_scan_filter_ = 
              new tf::MessageFilter<sensor_msgs::LaserScan>(
                    *laser_scan_sub_,
                    *tf_,
                    odom_frame_id_,
                    100);
  laser_scan_filter_->registerCallback(
                        boost::bind(
                          &SamplingNode::laserReceived, 
                          this, 
                          _1));
}

void
SamplingNode::RCCB()
{
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  delete laser_scan_filter_;
  ROS_INFO("reset laserReceived callback function");
  laser_scan_filter_ = 
              new tf::MessageFilter<sensor_msgs::LaserScan>(
                    *laser_scan_sub_,
                    *tf_,
                    odom_frame_id_,
                    100);
  laser_scan_filter_->registerCallback(
                        boost::bind(
                          &SamplingNode::laserReceived, 
                          this, 
                          _1));
}

/*
1. random move
2. ray-casting
3. claculate feature
4. record data
*/
void
SamplingNode::sampling()
{
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  if(!laserUpdated)
  {
    ROS_INFO("Doesn't receive any laser scan information. Waiting for 1 sec...");
    ros::Duration(1).sleep();
    return ;
  }
  // 1. random move
  //uniformly generate a Pose
  pf_vector_t rpose = MCL::uniformPoseGenerator((void*)map_);
  // 2. ray-casting
  amcl::AMCLLaserData ldata;
  SamplingNode::raycasting(lasers_[0], rpose, lrnum, lrmax, lrmin, lamin, lares, ldata);
  // 3. claculate feature
  //cache the features at the class member
  laser_feature_t lfeat = polygonCentroid(ldata);
  if(this->data_count_==0)
  {
    fxmin = lfeat.x; 
    fxmax = lfeat.x; 
    fymin = lfeat.y; 
    fymax = lfeat.y; 
    fdmin = lfeat.dist; 
    fdmax = lfeat.dist;
  }
  else
  {
    fxmin = std::min(fxmin, lfeat.x); 
    fxmax = std::max(fxmax, lfeat.x); 
    fymin = std::min(fymin, lfeat.y); 
    fymax = std::max(fymax, lfeat.y); 
    fdmin = std::min(fdmin, lfeat.dist); 
    fdmax = std::max(fdmax, lfeat.dist);
  }
  // 4. record data
  this->data_count_++;
  dataout_ptr_->writeALine(rpose, lfeat);
  if(data_count_ >= max_data_count_)
  {
    ros::shutdown();
  }
  return ;
}

void
SamplingNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  try{
    ROS_DEBUG("Enter SamplingNode::laserReceived");
    last_laser_received_ts_ = ros::Time::now();
    if( map_ == NULL ) {
      return;
    }
    int laser_index = -1;
  
    // Do we have the base->base_laser Tx yet?
    if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
    {
      ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
      lasers_.push_back(new amcl::AMCLLaser(*laser_));
      lasers_update_.push_back(true);
      laser_index = frame_to_laser_.size();
  
      tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                               tf::Vector3(0,0,0)),
                                   ros::Time(), laser_scan->header.frame_id);
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
    } else {
      // we have the laser pose, retrieve laser index
      laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }
  
    //convert laser_scan into LaserData
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
    ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);
  
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
    //update parameters every single time because there is possibility for reconfiguration.
    lrnum = laser_scan->ranges.size();
    lrmin = laser_scan->range_min;
    lrmax = laser_scan->range_max;
    lares = laser_scan->angle_increment;
    lamin = laser_scan->angle_min;
    lamax = laser_scan->angle_max;
    laserUpdated = true;
  }
  catch(const tf::TransformException& ex)
  {
    ROS_INFO("Failure %s", ex.what());
  }
  catch(const std::exception& e)
  {
    ROS_INFO("%s", e.what());
  }
}

SamplingNode::~SamplingNode()
{
  paramout_ptr_->writeALine(std::string("databinaryfile "), output_filename_data_);
  paramout_ptr_->writeALine(std::string("datacount "), data_count_);
  paramout_ptr_->writeALine(std::string("noise "), noise); 
  paramout_ptr_->writeALine(std::string("lrmin "), lrmin); 
  paramout_ptr_->writeALine(std::string("lrmax "), lrmax); 
  paramout_ptr_->writeALine(std::string("lares "), lares); 
  paramout_ptr_->writeALine(std::string("lamin "), lamin); 
  paramout_ptr_->writeALine(std::string("lamax "), lamax); 
  paramout_ptr_->writeALine(std::string("fxmin "), fxmin);  
  paramout_ptr_->writeALine(std::string("fxmax "), fxmax);  
  paramout_ptr_->writeALine(std::string("fymin "), fymin);  
  paramout_ptr_->writeALine(std::string("fymax "), fymax);  
  paramout_ptr_->writeALine(std::string("fdmin "), fdmin);  
  paramout_ptr_->writeALine(std::string("fdmax "), fdmax);  
  //scoped_ptr accounts for destructing paramout_ptr_ and dataout_ptr_
  //std::ofstream hold by them is deturcted in their destructors.
  delete laser_scan_filter_;
}
