#include "mixmcl/MixmclNode.h"
#include "mcl/MCL.cpp"
#include "amcl/pf/pf_resample.h"
template class MCL<MixmclNode>;
using namespace nuklei;
MixmclNode::MixmclNode() :
        MCL(),
        kdt_(NULL),
        first_reconfigureCB2_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
/////////////////////Dual MCL//////////////////
  //initialize kdtrees for sampling pose of dual MCL
  std::string param_key_name("default");
  if(!private_nh_.searchParam("feature_resolution_x", param_key_name))
    private_nh_.param("feature_resolution_x", fxres_, 10);
  else
    private_nh_.getParam(param_key_name.c_str(), fxres_);

  if(!private_nh_.searchParam("feature_resolution_y", param_key_name))
    private_nh_.param("feature_resolution_y", fyres_, 10);
  else
    private_nh_.getParam(param_key_name.c_str(), fyres_);

  if(!private_nh_.searchParam("feature_resolution_d", param_key_name))
    private_nh_.param("feature_resolution_d", fdres_, 10);
  else
    private_nh_.getParam(param_key_name.c_str(), fdres_);

  if(!private_nh_.searchParam("sample_param_filename", param_key_name))
    ROS_ERROR("In MixmclNode::MixmclNode() cannot find parameter named sample_param_filename");
  else
  {
    private_nh_.getParam(param_key_name.c_str(), sample_param_filename_);
    ROS_INFO("MixmclNode::MixmclNode() is going to read the parameter %s", sample_param_filename_.c_str());
  }

  if(!private_nh_.searchParam("dual_normalizer_ita", param_key_name))
    private_nh_.param("dual_normalizer_ita", ita_, 0.001);
  else
    private_nh_.param(param_key_name.c_str(), ita_, 0.001);

  if(!private_nh_.searchParam("mixing_rate", param_key_name))
    private_nh_.param("mixing_rate", mixing_rate_, 0.1);
  else
    private_nh_.param(param_key_name.c_str(), mixing_rate_, 0.5);

  if(!private_nh_.searchParam("dual_loc_bandwidth", param_key_name))
    private_nh_.param("dual_loc_bandwidth", loch_, 5.0);
  else
    private_nh_.param(param_key_name.c_str(), loch_, 0.1);

  if(!private_nh_.searchParam("dual_ori_bandwidth", param_key_name))
    private_nh_.param("dual_ori_bandwidth", orih_, 0.4);
  else
    private_nh_.param(param_key_name.c_str(), orih_, 0.5);

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

  createKCGrid();
/////////////////end Dual MCL//////////////////
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
      &MixmclNode::laserReceived,
      this, _1));

  particlecloud2_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud2", 2, true);

  dsrv2_ = new dynamic_reconfigure::Server<mixmcl::MIXMCLConfig>(ros::NodeHandle("~/mixmcl_dc"));
  dynamic_reconfigure::Server<mixmcl::MIXMCLConfig>::CallbackType cb2 = boost::bind(&MixmclNode::reconfigureCB2, this, _1, _2);
  dsrv2_->setCallback(cb2);
  this->printInfo();
}

void MixmclNode::RCCB()
{
  ROS_INFO("MixmclNode::RCCB() is called. Build density tree..");
  delete laser_scan_filter_;
  laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&MixmclNode::laserReceived,
                                                   this, _1));
}

void MixmclNode::reconfigureCB2(mixmcl::MIXMCLConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if(first_reconfigureCB2_call_)
  {
    first_reconfigureCB2_call_ = false;
    default_config2_ = config;
    return;
  }
/////////////////////Dual MCL//////////////////
  //reconfigure the mixing rate
  ROS_DEBUG("MixmclNode::reconfigureCB2(...)");
  ita_ = config.dual_normalizer_ita;
  mixing_rate_ = config.mixing_rate;
  loch_ = config.dual_loc_bandwidth;
  orih_ = config.dual_ori_bandwidth;

  //reinitialize kdtrees for dual MCL sampling
  if( fxres_!=config.feature_resolution_x || 
      fyres_!=config.feature_resolution_y || 
      fdres_!=config.feature_resolution_d ) 
  {
    fxres_ = config.feature_resolution_x;
    fyres_ = config.feature_resolution_y;
    fdres_ = config.feature_resolution_d;
    createKCGrid();
  }
/////////////////end Dual MCL//////////////////
}

MixmclNode::~MixmclNode()
{
  delete dsrv2_;
  delete laser_scan_filter_;
}

//move regular MCL samples in set_a with probability of 1-phi into set_b
void MixmclNode::mixtureProposals()
{
  const int set_a_idx = pf_->current_set;
  const int set_b_idx = (pf_->current_set + 1) % 2;
  pf_sample_set_t* set_a = pf_->sets + set_a_idx;
  pf_sample_set_t* set_b = pf_->sets + set_b_idx;
  pf_sample_t* sample_a;
  pf_sample_t* sample_b;
  set_b->sample_count = 0;
  double r;
  for(int i = 0; i < set_a->sample_count ; ++i)
  {
    r = MCL::rng_.uniform01();
    if(r > mixing_rate_)//equals to r < 1-mixing_rate_
    {
      sample_a = set_a->samples + i;
      sample_b = set_b->samples + set_b->sample_count++;
      sample_b->pose = sample_a->pose;
      sample_b->weight = sample_a->weight;
    }
  }

  //this variable would be used by some laser measurement models.
  set_b->converged = set_a->converged;

  //Now, set_b has the samples for regular MCL
  //Remaining samples for dual MCL stay in set_a
  //update sample count of set_a
  set_a->sample_count = set_a->sample_count - set_b->sample_count;

  //let regular MCL of set_b be the current set
  pf_->current_set = set_b_idx;
}

void
MixmclNode::createKCGrid()
{
  try
  {
    ROS_INFO("Building KCGrid... It might take a some time depending on file size");
    //if the file doesn't exist, it shall throw exception
    kcgrid_.reset(new KCGrid(fxres_, fyres_, fdres_, sample_param_filename_, mapx_, mapy_, loch_, orih_) );
    ROS_INFO("Finished building KCGrid.");
  }
  catch(const std::exception& e)
  {
    //there are three types of exception, std::out_of_range, std::runtime_error, and ios_base::failure
    //after catching the exception, shutdown
    ROS_FATAL("Cannot create KCGrid with the following parameters: fxres %d, fyres %d, fdres %d, parameter_file %s\ne.what: \n%s", fxres_, fyres_, fdres_, sample_param_filename_.c_str(), e.what());
    ros::shutdown();
  }

}

void MixmclNode::buildDensityTree(pf_t* pf, boost::shared_ptr<nuklei::KernelCollection>& kdt, double loch, double orih)
{
  if(!kdt)
    ROS_DEBUG("old kdt_ is NULL. Rebuilding density tree.");
  else
    ROS_DEBUG("old kdt_ is not NULL. Rebuilding density tree.");
  kdt.reset(new nuklei::KernelCollection);
  pf_sample_set_t* set = pf->sets + pf->current_set;
  for(int i=0;i<set->sample_count;i++)
  {
    tf::Quaternion q = tf::createQuaternionFromYaw(set->samples[i].pose.v[2]);
    //new kernel
    kernel::se3 se3k;
    se3k.loc_.X() = set->samples[i].pose.v[0];
    se3k.loc_.Y() = set->samples[i].pose.v[1];
    se3k.ori_.W() = q.w();
    se3k.ori_.X() = q.x();
    se3k.ori_.Y() = q.y();
    se3k.ori_.Z() = q.z();
    se3k.setWeight(set->samples[i].weight);
    //add the kernel into kdt_
    kdt->add(se3k);
  }
  //set kernel bandwidth to the tree
  kdt->setKernelLocH(loch);
  kdt->setKernelOriH(orih);
  kdt->normalizeWeights();
  kdt->buildKdTree();
}

void
MixmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  
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
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
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
  amcl::AMCLOdomData odata;
  //TODO remove inverse_odata
  amcl::AMCLOdomData inverse_odata;

  if(pf_init_)
  {
    // Compute change in pose
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);
    odata.pose = pose;
    odata.delta = delta;
    //inverse odata
    inverse_delta.v[0] = pf_odom_pose_.v[0] - pose.v[0];
    inverse_delta.v[1] = pf_odom_pose_.v[1] - pose.v[1];
    inverse_delta.v[2] = angle_diff(pf_odom_pose_.v[2], pose.v[2]);
    inverse_odata.pose = pose;
    inverse_odata.delta = inverse_delta;

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

  //before resample step, set_a_idx and set_b_idx will not be exchanged.
  const int set_a_idx = pf_->current_set;
  const int set_b_idx = (pf_->current_set + 1) % 2;

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;
    // Filter is now initialized
    pf_init_ = true;
    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;
    force_publication = true;
    resample_count_ = 0;
    //build a density tree based on set_a
    //because set_a is just initialized
    assert(pf_->sets[set_a_idx].sample_count!=0);//in case resample functions assign zero to the sample count
    buildDensityTree(pf_, kdt_, loch_, orih_);
    // using mixing_rate_ to seperate current set into two sets,
    // current set for regular MCL and another set for dual MCL
    mixtureProposals();
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //build a density tree based on set_b
    //set_a is resampled set
    //set_b is weighted set
    //before building the tree, let set_b takes account for odata
    pf_->current_set = set_b_idx;
    assert(pf_->sets[set_b_idx].sample_count!=0);//in case resample functions assign zero to the sample count
    odom_->UpdateAction(pf_, (amcl::AMCLSensorData*)&odata);
    buildDensityTree(pf_, kdt_, loch_, orih_);
    pf_->current_set = set_a_idx;
    // using mixing_rate_ to seperate current set into two sets,
    // current set for regular MCL and another set for dual MCL
    mixtureProposals();

    // Use the action data to update the filter
    // current set will be updated based on the odata
    odom_->UpdateAction(pf_, (amcl::AMCLSensorData*)&odata);
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    amcl::AMCLLaserData ldata;
    MCL::createLaserData(laser_index, ldata, laser_scan);
    //drawing samples from pre-built kernel density tree and current measurement model
    double total =  dualmclNEvaluation(ldata, inverse_odata);
    //publish the samples to particlecloud2 topic
    //note that this cloud has been applied the inverse odata.
    pf_->current_set = set_b_idx;
    MCL::publishParticleCloud(particlecloud2_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
    //Finally, combine the set together into set_a
    pf_sample_set_t* set_a = pf_->sets + set_a_idx;
    pf_sample_set_t* set_b = pf_->sets + set_b_idx;
    pf_sample_t* sample_a;
    pf_sample_t* sample_b;
    assert((set_a->sample_count + set_b->sample_count)<=max_particles_);
    for(int i = 0; i < set_b->sample_count ; ++i)
    {
      sample_a = set_a->samples + set_a->sample_count + i;
      sample_b = set_b->samples + i;
      sample_a->pose = sample_b->pose;
      sample_a->weight = sample_b->weight;
    }
    set_a->sample_count += set_b->sample_count;
    pf_->current_set = set_a_idx;
    double w_avg = pf_normalize(pf_, total);
    //TODO publish weighted particles to wpc_pub_
    //MCL::publishWeightedParticleCloud(wpc_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
    lasers_update_[laser_index] = false;
    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_) || 
        (force_publication ==true && sent_first_transform_ == false))
    {
      //pf_update_resample_lowvariance(pf_);
      //pf_update_resample_kld(pf_);
      resample_function_(pf_);
      resampled = true;
    }

    ROS_DEBUG("Num samples: %d\n", pf_->sets[pf_->current_set].sample_count);
    // Publish the resulting cloud
    if (!m_force_update) 
      MCL::publishParticleCloud(particlecloud_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
  }//endif(lasers_update_[laser_index])

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

//TODO remove inverse_odata
double MixmclNode::dualmclNEvaluation(amcl::AMCLLaserData& ldata, amcl::AMCLOdomData& inverse_odata)
{
  const int set_a_idx = pf_->current_set;
  const int set_b_idx = (pf_->current_set + 1 ) % 2;
  pf_sample_set_t* set_a = pf_->sets + set_a_idx;
  pf_sample_set_t* set_b = pf_->sets + set_b_idx;
  pf_sample_t* sample_a;
  pf_sample_t* sample_b;
  double dual_set_total = 0;
  pf_->current_set = set_a_idx;
  double regular_set_total = ldata.sensor->UpdateSensor(pf_, (amcl::AMCLSensorData*)&ldata);

  //Now, evaluation of regualr MCL has been performed for current set.
  //Start to perform Mixture MCL for another set.
  //First, based on ldata and pre-built density trees, generate samples and store them in set_b.
  //convert ldata to features, x, y, and dist.
  laser_feature_t feature = polygonCentroid(ldata);
  //get the corresponding tree from the pre-built density trees.
  stringstream ss;
  boost::shared_ptr<KernelCollection> tree = kcgrid_->getTree(feature.x, feature.y, feature.dist, ss);
  ROS_DEBUG_STREAM(ss);
  //drawing samples from the pre-built tree into set_b
  KernelCollection::const_sample_iterator iter = as_const(*(tree.get())).sampleBegin(set_b->sample_count);
  for(int i = 0; iter != iter.end(); ++iter, ++i)
  {
    // *iter returns a reference to a datapoint/kernel of tree
    // iter.index() returns the index (in tree) of that element.
    boost::shared_ptr<kernel::se3> se3_pose = (*iter).polySe3Sample();
    //convert kernel base se3_pose into pf_vecter_t.
    pf_vector_t vec_p;
    se3ToPose(*se3_pose, vec_p);
    set_b->samples[i].pose = vec_p;
    //Third, calculate importance factors for these samples.
    sample_b = set_b->samples + i;
    //convert pose to se3 and accees the noralizer ita_
    sample_b->weight = ita_ * kdt_->evaluationAt(*se3_pose);
    dual_set_total += sample_b->weight;
  }
  return dual_set_total + regular_set_total;
}

