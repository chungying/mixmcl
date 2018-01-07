#include "mixmcl/SamplingNode.h"
#include "mcl/MCL.cpp"
template class MCL<SamplingNode>;

SamplingNode::SamplingNode():
  MCL(),
  srv_name("/gazebo/set_model_state"),
  modelName("p3dx"),
  refFrame("world"),
  initFlag(false),
  randomSucceed(false),
  laserUpdated(false),
  dataout_ptr_(),
  paramout_ptr_(),
  dataCount(0)
{
   boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  //one text file for storing parameters and data file name.
  /*dataCount sampling number 
  **laser noise TODO 
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
  private_nh_.param("max_data_count", max_data_count, int(10000));
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

  //gazebo_msg::SetModelState
  cli_ = nh_.serviceClient<gazebo_msgs::SetModelState>(srv_name);
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;
  this->state.twist = twist;
  this->state.model_name = this->modelName;
  this->state.reference_frame = this->refFrame;
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

void
SamplingNode::moveRobotUniformly()
{
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_DEBUG("Execute SamplingNode::moveRobotUniformly()");
  //if laser is updated and if uniform pose is successfully initialized.
  if( randomSucceed && laserUpdated && initFlag)
  {
    ROS_DEBUG("store pose and features");
    //store features and pose
    pf_vector_t p4save = this->pose;
    laser_feature_t f4save = this->curFeature;
    /*ROS_INFO("rx: %f, ry: %f, rh: %f, lx: %f, ly: %f, ldist: %f", 
              p4save.v[0],
              p4save.v[1],
              p4save.v[2],
              f4save.x, 
              f4save.y, 
              f4save.dist);*/
    this->dataCount++;
    dataout_ptr_->writeALine(p4save, f4save);
    laserUpdated = false;
    randomSucceed = false;
    //TODO
    if(dataCount >= max_data_count)
    {
      ros::shutdown();
    }
  }

  //uniformly generate a Pose
  this->pose = MCL::uniformPoseGenerator((void*)map_);
  geometry_msgs::Pose gp;
  gp.position.x = pose.v[0];
  gp.position.y = pose.v[1];
  gp.position.z = 0;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(pose.v[2]),gp.orientation);
  this->state.pose = gp;

  gazebo_msgs::SetModelState setRobotModelPose;
  setRobotModelPose.request.model_state = this->state;
  if(cli_.call(setRobotModelPose))
  {
    //call service success
    ROS_DEBUG("successfully call SetModelState.");
    this->randomSucceed = true;
    if(!initFlag)initFlag = true;
  }
  else
  {
    //call service fail
    ROS_DEBUG("fail to call SetModelState.");
    initFlag = false;//if the service failed, make initFlag false to avoid using pose and ldata
  }
  ROS_DEBUG("model: %s, reference_frame: %s", this->modelName.c_str(), this->refFrame.c_str());
  ROS_DEBUG("generated pose: ( %.4f, %.4f, %.4f)", pose.v[0], pose.v[1], pose.v[2]);
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
    boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
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
      ROS_DEBUG("Received laser's pose wrt robot:"// %.3f %.3f %.3f",
               // lasers_[laser_index]->laser_pose.v[0],
               // lasers_[laser_index]->laser_pose.v[1],
               // lasers_[laser_index]->laser_pose.v[2]
                );
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
    ROS_DEBUG("converting ldata into features"); 
    //convert ldata into features
    //cache the features at the class member
    curFeature = polygonCentroid(ldata);
    //update parameters every single time because there is possibility for reconfiguration.
    lrmin = laser_scan->range_min;
    lrmax = laser_scan->range_max;
    lares = laser_scan->angle_increment;
    lamin = laser_scan->angle_min;
    lamax = laser_scan->angle_max;
    if(!initFlag)
    {
      fxmin = curFeature.x; 
      fxmax = curFeature.x; 
      fymin = curFeature.y; 
      fymax = curFeature.y; 
      fdmin = curFeature.dist; 
      fdmax = curFeature.dist;
    }
    else
    {
      fxmin = std::min(fxmin, curFeature.x); 
      fxmax = std::max(fxmax, curFeature.x); 
      fymin = std::min(fymin, curFeature.y); 
      fymax = std::max(fymax, curFeature.y); 
      fdmin = std::min(fdmin, curFeature.dist); 
      fdmax = std::max(fdmax, curFeature.dist);
    }
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
  paramout_ptr_->writeALine(std::string("datacount "), dataCount);
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
