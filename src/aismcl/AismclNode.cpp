#include <cmath> 
#include "aismcl/AismclNode.h"
#include "amcl/pf/pf_resample.h"
#include "mcl/MCL.cpp"
template class MCL<AismclNode>;

AismclNode::AismclNode() :
  MCL(),
  first_reconfigureCB2_call_(false),
  kdt_(NULL),
  dsrv2_(NULL),
  demc_params_(NULL)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
  /*
    ita_
    demc_params_
    loch_
    orih_
    particlecloud2_pub_
    dsrv2_
  */
  private_nh_.param("dual_normalizer_ita", ita_, 0.0001);
  if(!demc_params_)
    demc_params_.reset(new demc::demc_t);

  private_nh_.param("demc_factor_gamma",  demc_params_->gamma, 0.95);
  private_nh_.param("demc_loc_bandwidth", demc_params_->loc_bw, 0.01);
  private_nh_.param("demc_ori_bandwidth", demc_params_->ori_bw, 0.1);
  private_nh_.param("dual_loc_bandwidth", loch_, 10.0);
  private_nh_.param("dual_ori_bandwidth", orih_, 0.4);
  private_nh_.param("version1", version1_, true);
  private_nh_.param("static_update", static_update_, true);
  std::string tmp_resample_type;
  private_nh_.param("resample_type", tmp_resample_type, std::string("kld"));
  ROS_INFO("Resample type is %s", tmp_resample_type.c_str());
  if(tmp_resample_type == "augmented")
  {
    resample_function_ = &pf_update_resample_kld;
    ROS_INFO("Resample type: kld because MCMCL doesn't take augmented resampling methods.");
  }
  else if(tmp_resample_type == "kld")
    resample_function_ = &pf_update_resample_kld;
  else if(tmp_resample_type == "lowvariance")
    resample_function_ = &pf_update_resample_lowvariance;
  else
  {
    resample_function_ = &pf_update_resample_kld;
    ROS_INFO("Resample type: kld instead of %s", tmp_resample_type.c_str());
  }
//  mapx_.first = MAP_WXGX(map_, 0);
//  mapx_.second = MAP_WXGX(map_, map_->size_x); 
//  mapy_.first = MAP_WYGY(map_, 0);
//  mapy_.second = MAP_WYGY(map_, map_->size_y); 
//  map_rng_x_ = mapx_.second - mapx_.first;
//  map_rng_y_ = mapy_.second - mapy_.first;

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
      &AismclNode::laserReceived,
      this, _1));

  particlecloud2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud2", 2, true);
  particlecloud3_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud3", 2, true);

  dsrv2_ = new dynamic_reconfigure::Server<mixmcl::MCMCLConfig>(ros::NodeHandle("~/aismcl_dc"));
  dynamic_reconfigure::Server<mixmcl::MCMCLConfig>::CallbackType cb2 = boost::bind(&AismclNode::reconfigureCB2, this, _1, _2);
  dsrv2_->setCallback(cb2);
  if(!kdt_)
    MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
  ROS_DEBUG("AismclNode::AismclNode() finished.");
  this->printInfo();
}

AismclNode::~AismclNode()
{
  delete dsrv2_;
  delete laser_scan_filter_;
}

//build mixmcl::MCMCLConfig
void AismclNode::reconfigureCB2(mixmcl::MCMCLConfig& config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  //TODO Because Mixmcl, Mcmcl and this Aismcl all use the following codes, it is better to wrap the codes with a class.
  //     Let the three class inherit the wrapping class
  /***TODO to be wrapped***/
  if(!first_reconfigureCB2_call_)
  {
    ROS_INFO("first reconfigureCB2. Build density tree...");
    MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    default_config2_ = config;
    first_reconfigureCB2_call_ = true;
    return;
  }
  /*
    ita_
    demc_params_
    loch_ for kdt
    orih_ for kdt
  */
  ita_ = config.dual_normalizer_ita;
  demc_params_->gamma = config.demc_factor_gamma;
  demc_params_->loc_bw = config.demc_loc_bandwidth;
  demc_params_->ori_bw = config.demc_ori_bandwidth;
  loch_ = config.dual_loc_bandwidth;
  orih_ = config.dual_ori_bandwidth;
  ROS_INFO("AismclNode::reconfigureCB2(...)");
  ROS_INFO("dual_normalizer_ita: %lf", ita_);
  ROS_INFO("demc_factor_gamma: %lf", demc_params_->gamma);
  ROS_INFO("demc_loc_bandwidth: %lf", demc_params_->loc_bw);
  ROS_INFO("demc_ori_bandwidth: %lf", demc_params_->ori_bw);
  ROS_INFO("dual_loc_bandwidth: %lf", loch_);
  ROS_INFO("dual_ori_bandwidth: %lf", orih_);
  /***End to be wrapped***/
}

void AismclNode::RCCB()
{
  ROS_INFO("AismclNode::RCCB() is called. Build density tree..");
  MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
  delete laser_scan_filter_;
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        1);
  laser_scan_filter_->registerCallback(boost::bind(&AismclNode::laserReceived,
                                                   this, _1));
}

void 
AismclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) != frame_to_laser_.end())
  {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }
  else
  {
    // we don't have the laser pose, create a new one
    ROS_DEBUG("Setting up laser %d (frame_id=%s)", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
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

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!MCL::getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  pf_vector_t inverse_delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);
    inverse_delta.v[0] = pf_odom_pose_.v[0] - pose.v[0];
    inverse_delta.v[1] = pf_odom_pose_.v[1] - pose.v[1];
    inverse_delta.v[2] = angle_diff(pf_odom_pose_.v[2], pose.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;

    // Set the laser update flags
    if(update)
      for(unsigned int i=0; i < lasers_update_.size(); i++)
        lasers_update_[i] = true;
  }

  const int set_a_idx = pf_->current_set;
  const int set_b_idx = (pf_->current_set + 1) % 2;
  amcl::AMCLOdomData odata;
  bool force_publication = false;
  if(!pf_init_)
  {
    ROS_DEBUG("initialize pf");
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    odata.pose = pose;
    odata.delta = delta;
    odom_->UpdateAction(pf_, (amcl::AMCLSensorData*)&odata);
  }

  amcl::AMCLLaserData ldata;
  MCL::createLaserData(laser_index, ldata, laser_scan);

  bool resampled = false;
  if(lasers_update_[laser_index])
  {
    geometry_msgs::PoseArray accepted_cloud;
    accepted_cloud.header.stamp = laser_scan->header.stamp;
    accepted_cloud.header.frame_id = global_frame_id_;
    geometry_msgs::PoseArray rejected_cloud;
    rejected_cloud.header.stamp = laser_scan->header.stamp;
    rejected_cloud.header.frame_id = global_frame_id_;
    //TODO iteration_number_
    //AIS parameters
    double total = demc::metropolisRejectAndCalculateWeight(ldata, ita_, kdt_.get(), demc_params_.get(), mapx_, mapy_, map_rng_x_, map_rng_y_, MCL::rng_, pf_, accepted_cloud, rejected_cloud);
    MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    double w_avg = pf_normalize(pf_, total);
    //TODO publish weighted particles to wpc_pub_
    //MCL::publishWeightedParticleCloud(wpc_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
    // Resample the particles
    if(!(++resample_count_ % resample_interval_) || 
        (force_publication ==true && sent_first_transform_ == false))
    {
      //pf_update_resample_low_variance(pf_);
      //pf_update_resample_pure_KLD(pf_);
      resample_function_(pf_);
      resampled = true;
    }
    double accepted_rate =  (double)accepted_cloud.poses.size() / ((double)accepted_cloud.poses.size() + (double)rejected_cloud.poses.size());
    ROS_DEBUG("Accepted chains: %ld", accepted_cloud.poses.size());
    ROS_DEBUG("Accepted rate: %lf", accepted_rate);
    ROS_DEBUG("Num samples: %d", pf_->sets[pf_->current_set].sample_count);
    particlecloud2_pub_.publish(accepted_cloud);
    particlecloud3_pub_.publish(rejected_cloud);
    lasers_update_[laser_index] = false;
    pf_odom_pose_ = pose;
    //Publish the resulting cloud
    if (!m_force_update) 
      MCL::publishParticleCloud(particlecloud_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
  }//endif(lasers_update_[laser_index])
  else if(static_update_)
  {
    geometry_msgs::PoseArray accepted_cloud;
    accepted_cloud.header.stamp = laser_scan->header.stamp;
    accepted_cloud.header.frame_id = global_frame_id_;
    geometry_msgs::PoseArray rejected_cloud;
    rejected_cloud.header.stamp = laser_scan->header.stamp;
    rejected_cloud.header.frame_id = global_frame_id_;
    double total = demc::metropolisRejectAndCalculateWeight(ldata, ita_, kdt_.get(), demc_params_.get(), mapx_, mapy_, map_rng_x_, map_rng_y_, MCL::rng_, pf_, accepted_cloud, rejected_cloud);
    if(version1_) 
    {
      MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
      double w_avg = pf_normalize(pf_, total);
    //TODO publish weighted particles to wpc_pub_
      resample_function_(pf_);
    }
    //Publish the resulting cloud
    if (!m_force_update) 
      MCL::publishParticleCloud(particlecloud_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
    //TODO update cloud information without resampling
    particlecloud2_pub_.publish(accepted_cloud);
    particlecloud3_pub_.publish(rejected_cloud);
    double accepted_rate =  (double)accepted_cloud.poses.size() / ((double)accepted_cloud.poses.size() + (double)rejected_cloud.poses.size());
    ROS_DEBUG("Accepted chains: %ld", accepted_cloud.poses.size());
    ROS_DEBUG("Accepted rate: %lf", accepted_rate);
    ROS_DEBUG("Num samples: %d", pf_->sets[pf_->current_set].sample_count);
    //TODO metropolis with same weights
    //if(!(++resample_count_ % resample_interval_*10))
    //{
    //  ROS_DEBUG("Update cluster statistics.");
    //  pf_update_without_resample(pf_);
    //  resampled = true;
    //}
  }
  
  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];
      pose_pub_.publish(p);
      last_published_pose = p;

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              laser_scan->header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_DEBUG("Failed to subtract base to odom transform");
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      if (tf_broadcast_ == true)
      {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        ros::Time transform_expiration = (laser_scan->header.stamp +
                                          transform_tolerance_);
        tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                            transform_expiration,
                                            global_frame_id_, odom_frame_id_);
        this->tfb_->sendTransform(tmp_tf_stamped);
        ROS_DEBUG("Broadcast new transform.");
        sent_first_transform_ = true;
      }
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
      ROS_DEBUG("Broadcast new transform.");
    }

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }
}

