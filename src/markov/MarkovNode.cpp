#include <cstdint>
#include "markov/MarkovNode.h"
#include "mcl/MCL.cpp"
template class MCL<MarkovNode>;
using namespace std;

//matrix vertion using original motion model
double MarkovNode::UpdateOdomO(amcl::AMCLOdomData* ndata)
{
  //transform those matrices wrt each origin_pose in set_a
  //multipy all the element into the set_b grid
  //sum up those resulting matrices
  //assign the summation to origin_pose in set_a

  vector<double> ang_arr;
  for(int aidx = 0; aidx < size_a_;++aidx)
    ang_arr.push_back(IDX2ANG(aidx,ares_));
  int nb_threads_hint = std::thread::hardware_concurrency();
  int nb_threads = (nb_threads_hint == 0u ? 8u : nb_threads_hint);
  //local variables
  double delta_rot1, delta_trans, delta_rot2;
  double radius;
  int matsize;
  boost::shared_ptr<Matrix> X, Y;
  
  //update translation and rotations
  if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] + 
          ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
    delta_rot1 = 0.0;
  else
  {
    pf_vector_t old_pose = pf_vector_sub(ndata->pose, ndata->delta);
    delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]),old_pose.v[2]);
  }
  delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] +
                     ndata->delta.v[1]*ndata->delta.v[1]);
  delta_rot2 = angle_diff(ndata->delta.v[2], delta_rot1);
  //size_a_ matsize*matsize-matrices origin_pose to size_a_*matsize*matsize poses
  //update the 72 matrices for a base position (0,0) with headings at regular intervals of angular resolution ares_ from -M_PI to M_PI.
  //decide maximum and minimum of position X and Y
  //the difference from base position to each extremum is trans*(1+alpha3)
  radius = delta_trans*(1+odom_->alpha3*4);
  vector<double> side;
  for(double p = -map_->scale; p >= -radius ; p-=map_->scale)
    side.push_back(p);
  std::reverse(std::begin(side),std::end(side));
  for(double p = 0.0; p <= radius ; p+=map_->scale)
    side.push_back(p);
  matsize = side.size();
  X.reset(new Matrix());
  Y.reset(new Matrix());
  X->reserve(matsize*matsize);
  Y->reserve(matsize*matsize);
  for(int i = 0; i < matsize;++i)
  {
    for(int j = 0 ; j < matsize ; ++j)
    {
      X->push_back(side[i]);
      Y->push_back(side[j]);
    }
  }
  auto worker = [&ang_arr,X,Y,&delta_rot1,&delta_trans,&delta_rot2]
  (aIter beg_oa, aIter end_oa, mIter beg_m, mIter end_m, amcl::AMCLOdom* odom)
  {
    mIter mit = beg_m;
    for(aIter oait = beg_oa ; oait != end_oa ; ++oait, ++mit)
    {
      //making the index of the matrix's angle
      for(int maidx = 0 ; maidx < ang_arr.size() ; ++maidx)
      {
        Matrix& matrix = (*mit)[maidx];
        matrix.reserve(X->size());
        for(int i = 0; i < X->size();++i)
        {
          //calculate Tr, R1, R2
          double tran, rot1, rot2;
          odometry(0.0,0.0,(*oait),(*X)[i],(*Y)[i],ang_arr[maidx],rot1,tran,rot2);
          //calculate P
          double prob = motionModelO(odom, delta_rot1, delta_trans, delta_rot1, rot1, tran, rot2);
          matrix.push_back(prob);
        }
      }
    }
  };
  //create threads
  vector<std::thread> threads(nb_threads);
  aIter oait = std::begin(ang_arr);
  MatMatrices mat_prob_matrices(size_a_, VecMatrices(size_a_, Matrix()));//for storing size_a_*size_a_ matrices
  mIter mit = std::begin(mat_prob_matrices);
  int grainsize = (int)((double)ang_arr.size()/(double)nb_threads);
  for(auto tit = std::begin(threads); tit != std::end(threads)-1 ; ++tit)
  {
    *tit = std::thread(worker, oait, oait+grainsize, mit, mit+grainsize, odom_);
    oait+=grainsize;
    mit+=grainsize;
  }
  threads.back() = std::thread(worker,oait, std::end(ang_arr), mit, std::end(mat_prob_matrices), odom_);
  //wait threads finished
  for(auto&& thread: threads) {
    thread.join();
  }

  //update bel(xt-1,xt,action)
  //local variables
  pf_sample_set_t* set_b = grid_->sets + (grid_->current_set+1)%2;
  pf_sample_set_t* set_a = grid_->sets + grid_->current_set;
  double total_weight = 0;
  int sample_counter = 0;
  std::mutex worker2_mutex;
  /*common non-mutable input:
  set_a
  set_b
  X
  Y
  ang_arr
  mat_prob_matrices
  size_a_
  ares_
  map_
  mapidx2freeidx_
  */

  /*common mutable input:
  total_weight
  sample_counter
  std::mutex worker2_mutex;
  */
  /*parameters: iterators of active_sample_indices_
  vector<int>::iterator beg
  vector<int>::iterator end
  */
  int percent_count;
  auto worker2 = [&worker2_mutex, set_a, set_b, X, Y, &mat_prob_matrices, &ang_arr, &percent_count]
  (double& total_weight, int& sample_counter, int const total_sample, vector<int>::iterator sample_beg, vector<int>::iterator sample_end, int const size_a_, map_t const * map_, vector<vector<int> >& mapidx2freeidx_, int const ares_)
  {
    for(auto iter = sample_beg; iter != sample_end; ++iter)
    {
      vector<int> free_ngb_indices, local_ngb_indices;
      free_ngb_indices.reserve(X->size());
      local_ngb_indices.reserve(X->size());
      int origin_idx = *iter;
      pf_sample_t* sample_origin = set_b->samples + origin_idx;
      //create neighbor index lists, one for free_space, one for local
      //find valid neighbors according to position of sample_origin
      for(int nidx = 0; nidx < X->size(); ++nidx)
      {
        //get translated positions based on each pair in X and Y
        int ngb_map_idx_x = MAP_GXWX(map_,sample_origin->pose.v[0]+(*X)[nidx]);
        int ngb_map_idx_y = MAP_GYWY(map_,sample_origin->pose.v[1]+(*Y)[nidx]);
        //check if ngb_map_idx is valid
        if(MAP_VALID(map_,ngb_map_idx_x,ngb_map_idx_y)==false || (map_->cells[MAP_INDEX(map_,ngb_map_idx_x,ngb_map_idx_y)].occ_state != -1))
          continue;
        //save indices of valid neighbors
        free_ngb_indices.push_back(mapidx2freeidx_[ngb_map_idx_x][ngb_map_idx_y]);
        local_ngb_indices.push_back(nidx);
      }
      VecMatrices& vec_prob_matrices = mat_prob_matrices[ANG2IDX(sample_origin->pose.v[2], ares_)];
      double sample_origin_weight = 0.0;
      for(int maidx = 0; maidx < ang_arr.size(); ++maidx)
      {
        Matrix& motion_prob_mat = vec_prob_matrices[maidx];
        for(int idx = 0 ; idx < free_ngb_indices.size(); ++idx)
        {
          int sample_ngb_idx = free_ngb_indices[idx]*size_a_ + maidx;
          int local_ngb_idx = local_ngb_indices[idx];
          sample_origin_weight += set_b->samples[sample_ngb_idx].weight * motion_prob_mat[local_ngb_idx];
        }
      }
      set_a->samples[origin_idx].weight = sample_origin_weight;
      std::lock_guard<std::mutex> lg(worker2_mutex);
      total_weight += sample_origin_weight;
      ++sample_counter;
      if(sample_counter%percent_count == 0)
        ROS_DEBUG("progress: %f %d/%d",1.0*sample_counter/total_sample, sample_counter, total_sample);
    }
  };

  //create threads
  //join threads
  vector<std::thread> threads2(nb_threads);
  vector<int>::iterator sit = std::begin(active_sample_indices_);
  grainsize = (int)((double)active_sample_indices_.size()/(double)nb_threads);
  int total_sample = active_sample_indices_.size();
  percent_count = total_sample * 0.01;
  ROS_INFO("percent_count of total_sample: %d of %d", percent_count, total_sample);
  for(auto tit = std::begin(threads2); tit != std::end(threads2)-1 ; ++tit)
  {
    *tit = std::thread(worker2, std::ref(total_weight), std::ref(sample_counter), total_sample, sit, sit+grainsize, size_a_, map_, std::ref(mapidx2freeidx_), ares_);
    sit+=grainsize;
  }
  threads2.back() = std::thread(worker2, std::ref(total_weight), std::ref(sample_counter), total_sample, sit, std::end(active_sample_indices_), size_a_, map_, std::ref(mapidx2freeidx_), ares_);
  //wait threads finished
  for(auto&& thread: threads2) {
    thread.join();
  }

  //delete X;
  //delete Y;
  return total_weight;
}

int MarkovNode::downsizingSampling(pf_sample_set_t* set_a, pf_sample_set_t* set_b, int target_size)
{
  pf_sample_t *sample_a, *sample_b;
  double r,c,U;
  int m, i;
  double count_inv, total;
  
  count_inv = 1.0/target_size;
  total = 0;
  r = MCL<void>::rng_.uniform01() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  set_b->sample_count = 0;
  while(set_b->sample_count < target_size)
  {
    sample_b = set_b->samples + set_b->sample_count++;
    U = r + m * count_inv;
    while(U>c)
    {
      i++;
      if(i >= set_a->sample_count)
      {
        c = set_a->samples[0].weight;
        i = 0;
        m = 0;
        U = r + m * count_inv;
        continue;
      }
      c += set_a->samples[i].weight;
    }
    m++;
    sample_b->pose = set_a->samples[i].pose;
    sample_b->weight = 1.0;
    total += sample_b->weight;
    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
  }
  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  // Re-compute cluster statistics
  pf_cluster_stats(NULL, set_b);
}

void MarkovNode::initialMarkovGrid()
{
  //make a matrix for converting positions to indices of free_space_indices
  int map_x,map_y;
  for(int i = 0 ; i < free_space_indices.size(); ++i)
  {
    map_x = free_space_indices[i].first;
    map_y = free_space_indices[i].second;
    assert(map_x >=0);
    assert(map_y >=0);
    mapidx2freeidx_[map_x][map_y] = i;
  }
  //initialize particle grid
  
  grid_ = new pf_t;
  grid_->max_samples = max_particles_;
  grid_->min_samples = 0;
  grid_->pop_err = 0.0;
  grid_->pop_z = 0;
  grid_->dist_threshold = 0.0; 
  grid_->w_slow = 0.0;
  grid_->w_fast = 0.0;
  grid_->alpha_slow = 0.0;
  grid_->alpha_fast = 0.0;
  grid_->converged = 0; 
  
  grid_->current_set = 0;
  pf_sample_set_t *set;
  for(int s = 0 ; s < 2 ; ++s)
  {
    set = grid_->sets + s;
    set->sample_count = max_particles_;
    set->samples = new pf_sample_t[max_particles_];
    set->kdtree = NULL;
    set->cluster_count = 0;
    set->cluster_max_count = max_particles_;
    set->clusters = NULL;
    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
    set->converged = 0; 
  }
  //setting message metadata
  int free_space_no = free_space_indices.size();
  free_idcs_msg_.layout.dim.resize(2);
  free_idcs_msg_.layout.dim[0].label = "positional";
  free_idcs_msg_.layout.dim[0].size = free_space_no;
  free_idcs_msg_.layout.dim[0].stride = free_space_no*2;
  free_idcs_msg_.layout.dim[1].label = "xyindex";
  free_idcs_msg_.layout.dim[1].size = 2;
  free_idcs_msg_.layout.dim[1].stride = 2;
  free_idcs_msg_.data.resize(free_space_no*2);
  positions_msg_.layout.dim.resize(2);
  positions_msg_.layout.dim[0].label = "positional";
  positions_msg_.layout.dim[0].size = free_space_no;
  positions_msg_.layout.dim[0].stride = free_space_no*2;
  positions_msg_.layout.dim[1].label = "xycoord";
  positions_msg_.layout.dim[1].size = 2;
  positions_msg_.layout.dim[1].stride = 2;
  positions_msg_.data.resize(free_space_no*2);
  hist_layout_.dim.resize(2);
  hist_layout_.dim[0].label = "positional";
  hist_layout_.dim[0].size = free_space_no;
  hist_layout_.dim[0].stride = free_space_no*size_a_;
  hist_layout_.dim[1].label = "angular";
  hist_layout_.dim[1].size = size_a_;
  hist_layout_.dim[1].stride = size_a_;
  
  int sidx = 0;
  pf_sample_t *sample;
  for(int free_idx = 0; free_idx < free_space_indices.size(); ++free_idx)
  {
    double position_x = MAP_WXGX(map_, free_space_indices[free_idx].first);
    double position_y = MAP_WYGY(map_, free_space_indices[free_idx].second);
    int map_idx_x = MAP_GXWX(map_,position_x);
    int map_idx_y = MAP_GYWY(map_,position_y);
    free_idcs_msg_.data[free_idx * 2] = free_space_indices[free_idx].first;
    free_idcs_msg_.data[free_idx*2+1] = free_space_indices[free_idx].second;
    positions_msg_.data[free_idx * 2] = position_x;
    positions_msg_.data[free_idx*2+1] = position_y;

    int free_space_idx = mapidx2freeidx_[map_idx_x][map_idx_y];
    assert(free_space_idx == free_idx);
    for(int a = 0 ; a < size_a_;++a)
    {
      double ang = IDX2ANG(a, ares_);;
      assert((free_space_idx * size_a_ + ANG2IDX(ang, ares_)) == sidx);
      for(int s = 0 ; s < 2 ; ++s)
      {
        sample = grid_->sets[s].samples + sidx;
        sample->pose.v[0] = position_x;
        sample->pose.v[1] = position_y;
        sample->pose.v[2] = ang;
        sample->weight = 1.0 / max_particles_;
      }
      ++sidx;
    }
  }
  //for(int s = 0 ; s < 2 ; ++s)
  //{
  //  int sidx = 0;
  //  for(std::pair<int,int> p:free_space_indices)
  //  {
  //    for(int a = 0 ; a < size_a_;++a)
  //    {
  //      sample = set->samples + sidx;
  //      sample->pose.v[0] = MAP_WXGX(map_, p.first);
  //      sample->pose.v[1] = MAP_WYGY(map_, p.second);
  //      sample->pose.v[2] = IDX2ANG(a, ares_);
  //      int free_space_idx = mapidx2freeidx_[MAP_GXWX(map_,sample->pose.v[0])][MAP_GYWY(map_,sample->pose.v[1])];
  //      int ang_idx = ANG2IDX(sample->pose.v[2], ares_);
  //      int sample_idx = free_space_idx * size_a_ + ang_idx;
  //      assert(sample_idx == sidx);
  //      sample->weight = 1.0 / max_particles_;
  //      ++sidx;
  //    }
  //    set->kdtree = NULL;
  //    set->cluster_count = 0;
  //    set->cluster_max_count = max_particles_;
  //    set->clusters = NULL;
  //    set->mean = pf_vector_zero();
  //    set->cov = pf_matrix_zero();
  //    set->converged = 0; 
  //  }
  //}
}

MarkovNode::~MarkovNode(){
  ROS_DEBUG("MarkovNode::~MarkovNode()");
  delete laser_scan_filter_;
  delete[] grid_->sets[0].samples;
  delete[] grid_->sets[1].samples;
  delete grid_;
}
MarkovNode::MarkovNode(): MCL(),
  grid_(NULL),
  mapidx2freeidx_(map_->size_x,std::vector<int>(map_->size_y, -1))
{
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  ROS_DEBUG("MarkovNode::MarkovNode() is allocating laser_scan_filter_.");
  //maximize the buffer of laserReceive
  private_nh_.param("laser_buffer_size", laser_buffer_size_, 500);
  private_nh_.param("angular_resolution", ares_, 5);
  private_nh_.param("cloud_size", cloud_size_, 10000);
  private_nh_.param("odom_update_radius", radius_, 3.0);
  size_a_ = (int)(360.0/ares_);
  max_particles_ = free_space_indices.size() * size_a_;
  epson_ = 1.0/max_particles_;
  active_sample_indices_.reserve(max_particles_);
  pf_free( pf_ );
  pf_ = pf_alloc(min_particles_, cloud_size_,//for sampling from grid_
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)MCL::uniformPoseGenerator,
                 (void *)map_);
  this->laser_scan_filter_ = 
    new tf::MessageFilter<sensor_msgs::LaserScan>(
              *laser_scan_sub_, 
              *tf_, 
              odom_frame_id_, 
              laser_buffer_size_);
  ROS_DEBUG("MarkovNode::MarkovNode() is registering callback function to  laser_scan_filter_.");
  this->laser_scan_filter_->registerCallback(
              boost::bind(&MarkovNode::laserReceived,
              this, _1));

  //setting up publishers
  histograms_pub_ = private_nh_.advertise<stamped_std_msgs::StampedFloat64MultiArray>("/histograms",1);
  positions_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("/positions",1);
  indices_pub_ = private_nh_.advertise<std_msgs::UInt16MultiArray>("/indices",1);

  ROS_DEBUG("MarkovNode::MarkovNode() has successfully reset laser_scan_filter_.");
  //disable global localization
  ROS_INFO("Shutting down global localization service");
  global_loc_srv_.shutdown();
  //disable laser received check
  check_laser_timer_.stop();
  ROS_INFO("Successfully shut down global localization service and laser timer");
  //initial particle grid
  initialMarkovGrid();
}

void
MarkovNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
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
    //ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
    //          lasers_[laser_index]->laser_pose.v[0],
    //          lasers_[laser_index]->laser_pose.v[1],
    //          lasers_[laser_index]->laser_pose.v[2]);
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

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

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

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is initialized
    pf_init_ = true;

    // Should update sensor data
    for(unsigned int i=0; i < lasers_update_.size(); i++)
      lasers_update_[i] = true;

    force_publication = true;

    resample_count_ = 0;

    grid_->current_set = (grid_->current_set + 1)%2;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && lasers_update_[laser_index])
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    amcl::AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;
    // Use the action data to update the filter
    //odom_->UpdateAction(pf_, (amcl::AMCLSensorData*)&odata);
    //requires grid_ current set and previous set
    ros::Time beg_odom = ros::Time::now();
    //implement UpdataSensor in MarkovNode
    //double totalweight = odom_->UpdateSensor(grid_, (amcl::AMCLSensorData*)&odata);
    ROS_DEBUG("begin original odometry update. current_set:%d\n",grid_->current_set);
    double totalweight = UpdateOdomO(&odata);
    ROS_DEBUG("finished original odometry update. It takes %f\n", (ros::Time::now() - beg_odom).toSec());
    //normalization of weight
    double w_avg = pf_normalize(grid_, totalweight);
    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(lasers_update_[laser_index])
  {
    pf_sample_set_t* set;
    amcl::AMCLLaserData ldata;
    MCL::createLaserData(laser_index, ldata, laser_scan);
    //requires grid_ current set
    ROS_DEBUG("begin laser update. current_set:%d\n",grid_->current_set);
    ros::Time beg_laser = ros::Time::now();
    double total = lasers_[laser_index]->UpdateSensor(grid_, (amcl::AMCLSensorData*)&ldata);
    ROS_DEBUG("finished laser update. It takes %f\n", (ros::Time::now() - beg_laser).toSec());
    double w_avg = pf_normalize(grid_, total);
    set = grid_->sets + grid_->current_set;
    int sample_count = set->sample_count;
    active_sample_indices_.clear();
    stamped_std_msgs::StampedFloat64MultiArray hist_msg;
    hist_msg.header.frame_id = global_frame_id_;
    hist_msg.header.stamp = laser_scan->header.stamp;
    hist_msg.array.layout = hist_layout_;
    hist_msg.array.data.resize(sample_count);
    for(int idx=0; idx < sample_count;++idx)
    {
      hist_msg.array.data[idx] = set->samples[idx].weight;
      if(set->samples[idx].weight > epson_ )
        active_sample_indices_.push_back(idx);
    }
    histograms_pub_.publish(hist_msg);
    if(resample_count_<1)
    {
      positions_pub_.publish(positions_msg_);
      indices_pub_.publish(free_idcs_msg_);
    }
    ROS_DEBUG("Num samples whose weight larger than %e: %ld/%d\n", epson_, active_sample_indices_.size(), max_particles_);
    //pf_update_augmented_weight(pf_, w_avg);

    lasers_update_[laser_index] = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      downsizingSampling(grid_->sets+grid_->current_set, pf_->sets+pf_->current_set, cloud_size_);
      //resample_function_(pf_);
      resampled = true;
    }
    //make current set as previous set
    grid_->current_set = (grid_->current_set + 1)%2;
    set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    if (!m_force_update) {
      MCL::publishParticleCloud(particlecloud_pub_, global_frame_id_, laser_scan->header.stamp, pf_);
    }
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

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

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

    //disable save pose part
    // Is it time to save our last pose to the param server
    //ros::Time now = ros::Time::now();
    //if((save_pose_period.toSec() > 0.0) &&
    //   (now - save_pose_last_time) >= save_pose_period)
    //{
    //  this->savePoseToServer();
    //  save_pose_last_time = now;
    //}
  }

}

//the version with squared translation and rotations
double MarkovNode::motionModelS(const pf_sample_t* sample_a, const pf_sample_t* sample_b, const amcl::AMCLOdom* odom, const double delta_rot1, const double delta_trans, const double delta_rot2)
{
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double a1,a2,a3;
  double b1,b2,b3;
  pf_vector_t delta;
  delta.v[0] = sample_a->pose.v[0]-sample_b->pose.v[0];
  delta.v[1] = sample_a->pose.v[1]-sample_b->pose.v[1];
  //intput sample_a sample_b ndata, output weight
  //calculate delta of sample_a and sample_b
  delta_trans_hat = sqrt(delta.v[0]*delta.v[0] +
                     delta.v[1]*delta.v[1]);
  if(delta_trans_hat < 0.01)
    delta_rot1_hat = 0.0;
  else
    delta_rot1_hat = angle_diff(atan2(delta.v[1], delta.v[0]),
                            sample_b->pose.v[2]);

  a1 = delta_rot1 - delta_rot1_hat; 
  b1 = odom->alpha1*delta_rot1_hat*delta_rot1_hat + 
      odom->alpha2*delta_trans_hat*delta_trans_hat;
  //when a is larger than 4*sqrt(b), where sqrt(b) is standard deviation
  if(b1 != 0 && a1*a1>=16.0*b1)
    return 0.0;

  delta.v[2] = angle_diff(sample_a->pose.v[2],sample_b->pose.v[2]);
  delta_rot2_hat = angle_diff(delta.v[2], delta_rot1_hat);

  a2 = delta_trans - delta_trans_hat; 
  b2 = odom->alpha3*delta_trans_hat*delta_trans_hat + 
      odom->alpha4*delta_rot1_hat*delta_rot1_hat +
      odom->alpha4*delta_rot2_hat*delta_rot2_hat;
  a3 = delta_rot2 - delta_rot2_hat; 
  b3 = odom->alpha1*delta_rot2_hat*delta_rot2_hat + 
      odom->alpha2*delta_trans_hat*delta_trans_hat;
  if(a2*a2>=16*b2 || a3*a3>=16*b3 )
    return 0.0;
  return pf_normal_distribution(a1,b1)*
         pf_normal_distribution(a2,b2)*
         pf_normal_distribution(a3,b3);
}

void MarkovNode::odometry(const double oldx, const double oldy, const double olda, const double newx, const double newy, const double newa, double& delta_rot1_hat, double& delta_trans_hat, double& delta_rot2_hat)
{
  double delta_x = newx-oldx;
  double delta_y = newy-oldy;
  double delta_a = angle_diff(newa,olda);
  delta_trans_hat = sqrt(delta_x*delta_x +
                     delta_y*delta_y);
  if(delta_trans_hat < 0.01)
    delta_rot1_hat = 0.0;
  else
    delta_rot1_hat = angle_diff(atan2(delta_y, delta_x),olda);
  delta_rot2_hat = angle_diff(delta_a, delta_rot1_hat);
}

//the original version in Probabilistic Robotics
double MarkovNode::motionModelO(const amcl::AMCLOdom* odom, const double delta_rot1, const double delta_trans, const double delta_rot2, const double delta_rot1_hat, const double delta_trans_hat, const double delta_rot2_hat)
{
  double a1 = delta_rot1 - delta_rot1_hat; 
  double b1 = odom->alpha1*fabs(delta_rot1_hat) + 
              odom->alpha2*delta_trans_hat;
  double a2 = delta_trans - delta_trans_hat; 
  double b2 = odom->alpha3*delta_trans_hat + 
              odom->alpha4*fabs(delta_rot1_hat) +
              odom->alpha4*fabs(delta_rot2_hat);
  double a3 = delta_rot2 - delta_rot2_hat; 
  double b3 = odom->alpha1*fabs(delta_rot2_hat) + 
              odom->alpha2*delta_trans_hat;
  return pf_normal_distribution(a1,b1)*
         pf_normal_distribution(a2,b2)*
         pf_normal_distribution(a3,b3);
}

