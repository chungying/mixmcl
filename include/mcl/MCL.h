#ifndef MCL_H
#define MCL_H
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"

#include "random_numbers/random_numbers.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

// Allows AMCL to run from bag file
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define NEW_UNIFORM_SAMPLING 1

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

static const std::string scan_topic_ = "scan";

template<class Derived>
class MCL 
{
  protected:
    MCL();
  public:
    ~MCL();

    /**
     * @brief Uses TF and LaserScan messages from bag file to drive AMCL instead
     * invoked in main 
     */
    void runFromBag(const std::string &in_bag_fn);

    int process();
    void savePoseToServer();

    static random_numbers::RandomNumberGenerator rng_;

  protected:
    tf::TransformBroadcaster* tfb_;

    // Use a child class to get access to tf2::Buffer class inside of tf_
    struct TransformListenerWrapper : public tf::TransformListener
    {
      inline tf2_ros::Buffer &getBuffer() {return tf2_buffer_;}
    };

    TransformListenerWrapper* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif

    static inline double getYaw(tf::Pose& t);

    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool setMapCallback(nav_msgs::SetMap::Request& req,
                        nav_msgs::SetMap::Response& res);

    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void applyInitialPose();

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    ros::Subscriber initial_pose_sub_;
    std::vector< amcl::AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    amcl::AMCLOdom* odom_;
    amcl::AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    // For slowing play-back when reading directly from a bag file
    ros::WallDuration bag_scan_period_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    amcl::odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    amcl::laser_model_t laser_model_type_;
    bool tf_broadcast_;

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);

    //pure virtual
    virtual void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan) = 0;
    virtual void GLCB() = 0;
    virtual void AIP() = 0;
    virtual void RCCB() = 0;
};

template<class D>
std::vector<std::pair<int,int> > MCL<D>::free_space_indices;
template<class D>
random_numbers::RandomNumberGenerator MCL<D>::rng_;

//template<class D>
//MCL<D>::MCL() :
//    sent_first_transform_(false),
//    latest_tf_valid_(false),
//    map_(NULL),
//    pf_(NULL),
//    resample_count_(0),
//    odom_(NULL),
//    laser_(NULL),
//    private_nh_("~"),
//    initial_pose_hyp_(NULL),
//    first_map_received_(false),
//    laser_scan_filter_(NULL),
//    first_reconfigure_call_(true)
//{
//  boost::recursive_mutex::scoped_lock l(configuration_mutex_);
//  // Grab params off the param server
//  private_nh_.param("use_map_topic", use_map_topic_, false);
//  private_nh_.param("first_map_only", first_map_only_, false);
//
//  double tmp;
//  private_nh_.param("gui_publish_rate", tmp, -1.0);
//  gui_publish_period = ros::Duration(1.0/tmp);
//  private_nh_.param("save_pose_rate", tmp, 0.5);
//  save_pose_period = ros::Duration(1.0/tmp);
//  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
//  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
//  private_nh_.param("laser_max_beams", max_beams_, 30);
//  private_nh_.param("min_particles", min_particles_, 100);
//  private_nh_.param("max_particles", max_particles_, 5000);
//  private_nh_.param("kld_err", pf_err_, 0.01);
//  private_nh_.param("kld_z", pf_z_, 0.99);
//  private_nh_.param("odom_alpha1", alpha1_, 0.2);
//  private_nh_.param("odom_alpha2", alpha2_, 0.2);
//  private_nh_.param("odom_alpha3", alpha3_, 0.2);
//  private_nh_.param("odom_alpha4", alpha4_, 0.2);
//  private_nh_.param("odom_alpha5", alpha5_, 0.2);
//  private_nh_.param("do_beamskip", do_beamskip_, false);
//  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
//  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
//  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);
//  private_nh_.param("laser_z_hit", z_hit_, 0.95);
//  private_nh_.param("laser_z_short", z_short_, 0.1);
//  private_nh_.param("laser_z_max", z_max_, 0.05);
//  private_nh_.param("laser_z_rand", z_rand_, 0.05);
//  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
//  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
//  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
//  std::string tmp_model_type;
//  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
//  if(tmp_model_type == "beam")
//    laser_model_type_ = amcl::LASER_MODEL_BEAM;
//  else if(tmp_model_type == "likelihood_field")
//    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
//  else if(tmp_model_type == "likelihood_field_prob"){
//    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB;
//  }
//  else
//  {
//    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
//             tmp_model_type.c_str());
//    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
//  }
//  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
//  if(tmp_model_type == "diff")
//    odom_model_type_ = amcl::ODOM_MODEL_DIFF;
//  else if(tmp_model_type == "omni")
//    odom_model_type_ = amcl::ODOM_MODEL_OMNI;
//  else if(tmp_model_type == "diff-corrected")
//    odom_model_type_ = amcl::ODOM_MODEL_DIFF_CORRECTED;
//  else if(tmp_model_type == "omni-corrected")
//    odom_model_type_ = amcl::ODOM_MODEL_OMNI_CORRECTED;
//  else
//  {
//    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
//             tmp_model_type.c_str());
//    odom_model_type_ = amcl::ODOM_MODEL_DIFF;
//  }
//  private_nh_.param("update_min_d", d_thresh_, 0.2);
//  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
//  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
//  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
//  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
//  private_nh_.param("resample_interval", resample_interval_, 2);
//  double tmp_tol;
//  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
//  transform_tolerance_.fromSec(tmp_tol);
//  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
//  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
//  private_nh_.param("tf_broadcast", tf_broadcast_, true);
//  {
//    double bag_scan_period;
//    private_nh_.param("bag_scan_period", bag_scan_period, -1.0);
//    bag_scan_period_.fromSec(bag_scan_period);
//  }
//
//  MCL::updatePoseFromServer();
//
//  //generic publication
//  cloud_pub_interval.fromSec(1.0);
//  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose", 2, true);
//  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
//
//  //generic services
//  //nomotionUpdateCallback is generic
//  nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &MCL::nomotionUpdateCallback, this);
//  set_map_srv_= nh_.advertiseService("set_map", &MCL::setMapCallback, this);
//
//  //generic subscribers
//  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &MCL::initialPoseReceived, this);
//
//  if(use_map_topic_) {
//    map_sub_ = nh_.subscribe("map", 1, &MCL::mapReceived, this);
//    ROS_INFO("Subscribed to map topic.");
//  } else {
//    MCL::requestMap();
//  }
//  m_force_update = false;
//
//  tfb_ = new tf::TransformBroadcaster();
//  tf_ = new TransformListenerWrapper();
////TODO avoid this in template class
//  //laser_scan_filter_ = 
//  //  new tf::MessageFilter<sensor_msgs::LaserScan>(
//  //            *laser_scan_sub_, 
//  //            *tf_, 
//  //            odom_frame_id_, 
//  //            100);
//  //D* dptr = static_cast<D*>(this);
//  //laser_scan_filter_->registerCallback( boost::bind(&D::laserReceived, dptr, _1));
//
//  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
//  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&MCL<D>::reconfigureCB, this, _1, _2);
//  dsrv_->setCallback(cb);
//
//  // 15s timer to warn on lack of receipt of laser scans, #5209
//  laser_check_interval_ = ros::Duration(15.0);
//  check_laser_timer_ = nh_.createTimer(laser_check_interval_, 
//                                       boost::bind(&MCL<D>::checkLaserReceived, this, _1));
//}
//
//template<class D>
//void 
//MCL<D>::reconfigureCB(amcl::AMCLConfig &config, uint32_t level)
//{
//  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
//
//  //we don't want to do anything on the first call
//  //which corresponds to startup
//  if(first_reconfigure_call_)
//  {
//    first_reconfigure_call_ = false;
//    default_config_ = config;
//    return;
//  }
//
//  if(config.restore_defaults) {
//    config = default_config_;
//    //avoid looping
//    config.restore_defaults = false;
//  }
//
//  d_thresh_ = config.update_min_d;
//  a_thresh_ = config.update_min_a;
//
//  resample_interval_ = config.resample_interval;
//
//  laser_min_range_ = config.laser_min_range;
//  laser_max_range_ = config.laser_max_range;
//
//  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
//  save_pose_period = ros::Duration(1.0/config.save_pose_rate);
//
//  transform_tolerance_.fromSec(config.transform_tolerance);
//
//  max_beams_ = config.laser_max_beams;
//  alpha1_ = config.odom_alpha1;
//  alpha2_ = config.odom_alpha2;
//  alpha3_ = config.odom_alpha3;
//  alpha4_ = config.odom_alpha4;
//  alpha5_ = config.odom_alpha5;
//
//  z_hit_ = config.laser_z_hit;
//  z_short_ = config.laser_z_short;
//  z_max_ = config.laser_z_max;
//  z_rand_ = config.laser_z_rand;
//  sigma_hit_ = config.laser_sigma_hit;
//  lambda_short_ = config.laser_lambda_short;
//  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;
//
//  if(config.laser_model_type == "beam")
//    laser_model_type_ = amcl::LASER_MODEL_BEAM;
//  else if(config.laser_model_type == "likelihood_field")
//    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD;
//  else if(config.laser_model_type == "likelihood_field_prob")
//    laser_model_type_ = amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB;
//
//  if(config.odom_model_type == "diff")
//    odom_model_type_ = amcl::ODOM_MODEL_DIFF;
//  else if(config.odom_model_type == "omni")
//    odom_model_type_ = amcl::ODOM_MODEL_OMNI;
//  else if(config.odom_model_type == "diff-corrected")
//    odom_model_type_ = amcl::ODOM_MODEL_DIFF_CORRECTED;
//  else if(config.odom_model_type == "omni-corrected")
//    odom_model_type_ = amcl::ODOM_MODEL_OMNI_CORRECTED;
//
//  if(config.min_particles > config.max_particles)
//  {
//    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
//    config.max_particles = config.min_particles;
//  }
//
//  min_particles_ = config.min_particles;
//  max_particles_ = config.max_particles;
//  alpha_slow_ = config.recovery_alpha_slow;
//  alpha_fast_ = config.recovery_alpha_fast;
//  tf_broadcast_ = config.tf_broadcast;
//
//  do_beamskip_= config.do_beamskip; 
//  beam_skip_distance_ = config.beam_skip_distance; 
//  beam_skip_threshold_ = config.beam_skip_threshold; 
//
//  pf_ = pf_alloc(min_particles_, max_particles_,
//                 alpha_slow_, alpha_fast_,
//                 (pf_init_model_fn_t)MCL::uniformPoseGenerator,
//                 (void *)map_);
//  pf_err_ = config.kld_err; 
//  pf_z_ = config.kld_z; 
//  pf_->pop_err = pf_err_;
//  pf_->pop_z = pf_z_;
//
//  // Initialize the filter
//  pf_vector_t pf_init_pose_mean = pf_vector_zero();
//  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
//  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
//  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
//  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
//  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
//  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
//  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
//  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
//  pf_init_ = false;
//
//  // Instantiate the sensor objects
//  // Odometry
//  delete odom_;
//  odom_ = new amcl::AMCLOdom();
//  ROS_ASSERT(odom_);
//  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
//  // Laser
//  delete laser_;
//  laser_ = new amcl::AMCLLaser(max_beams_, map_);
//  ROS_ASSERT(laser_);
//  if(laser_model_type_ == amcl::LASER_MODEL_BEAM)
//  {
//    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
//                         sigma_hit_, lambda_short_, 0.0);
//  }
//  else if(laser_model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB)
//  {
//    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
//    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
//					laser_likelihood_max_dist_, 
//					do_beamskip_, beam_skip_distance_, 
//					beam_skip_threshold_, beam_skip_error_threshold_);
//    ROS_INFO("Done initializing likelihood field model with probabilities.");
//  }
//  else if(laser_model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD)
//  {
//    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
//    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
//                                    laser_likelihood_max_dist_);
//    ROS_INFO("Done initializing likelihood field model.");
//  }
//
//  odom_frame_id_ = config.odom_frame_id;
//  base_frame_id_ = config.base_frame_id;
//  global_frame_id_ = config.global_frame_id;
//
//  //move build_density_tree to Derived::reconfigureCB()
//  static_cast<D*>(this)->RCCB();
//
//  //delete laser_scan_filter_;
//  //laser_scan_filter_ = 
//  //        new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
//  //                                                      *tf_, 
//  //                                                      odom_frame_id_, 
//  //                                                      100);
//  //laser_scan_filter_->registerCallback(boost::bind(&D::laserReceived,
//  //                                                 this, _1));
//
//}
//
//// force nomotion updates (amcl updating without requiring motion)
//template<class D>
//bool 
//MCL<D>::nomotionUpdateCallback(std_srvs::Empty::Request& req,
//                            std_srvs::Empty::Response& res)
//{
//  m_force_update = true;
//  //ROS_INFO("Requesting no-motion update");
//  return true;
//}
//
//template<class D>
//bool
//MCL<D>::setMapCallback(nav_msgs::SetMap::Request& req,
//                    nav_msgs::SetMap::Response& res)
//{
//  MCL::handleMapMessage(req.map);
//  MCL::handleInitialPoseMessage(req.initial_pose);
//  res.success = true;
//  return true;
//}
//
//template<class D>
//void
//MCL<D>::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
//{
//  MCL::handleInitialPoseMessage(*msg);
//}
//
//template<class D>
//void
//MCL<D>::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
//{
//  if( first_map_only_ && first_map_received_ ) {
//    return;
//  }
//
//  MCL::handleMapMessage( *msg );
//
//  first_map_received_ = true;
//}
//
//template<class D>
//void
//MCL<D>::requestMap()
//{
//  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);
//  // get map via RPC
//  nav_msgs::GetMap::Request  req;
//  nav_msgs::GetMap::Response resp;
//  ROS_INFO("Requesting the map...");
//  while(!ros::service::call("static_map", req, resp))
//  {
//    ROS_WARN("Request for map failed; trying again...");
//    ros::Duration d(0.5);
//    d.sleep();
//  }
//  handleMapMessage( resp.map );
//}
//
//template<class D>
//void
//MCL<D>::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
//{
//  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
//
//  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
//           msg.info.width,
//           msg.info.height,
//           msg.info.resolution);
//
//  freeMapDependentMemory();
//  // Clear queued laser objects because they hold pointers to the existing
//  // map, #5202.
//  lasers_.clear();
//  lasers_update_.clear();
//  frame_to_laser_.clear();
//
//  map_ = convertMap(msg);
//
//#if NEW_UNIFORM_SAMPLING
//  // Index of free space
//  MCL::free_space_indices.resize(0);
//  for(int i = 0; i < map_->size_x; i++)
//    for(int j = 0; j < map_->size_y; j++)
//      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
//        MCL::free_space_indices.push_back(std::make_pair(i,j));
//#endif
//  // Create the particle filter
//  pf_ = pf_alloc(min_particles_, max_particles_,
//                 alpha_slow_, alpha_fast_,
//                 (pf_init_model_fn_t)MCL::uniformPoseGenerator,
//                 (void *)map_);
//  pf_->pop_err = pf_err_;
//  pf_->pop_z = pf_z_;
//
//  // Initialize the filter
//  updatePoseFromServer();
//  pf_vector_t pf_init_pose_mean = pf_vector_zero();
//  pf_init_pose_mean.v[0] = init_pose_[0];
//  pf_init_pose_mean.v[1] = init_pose_[1];
//  pf_init_pose_mean.v[2] = init_pose_[2];
//  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
//  pf_init_pose_cov.m[0][0] = init_cov_[0];
//  pf_init_pose_cov.m[1][1] = init_cov_[1];
//  pf_init_pose_cov.m[2][2] = init_cov_[2];
//  //update the filter
//  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
//  pf_init_ = false;
//
//  // Instantiate the sensor objects
//  // Odometry
//  delete odom_;
//  odom_ = new amcl::AMCLOdom();
//  ROS_ASSERT(odom_);
//  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
//  // Laser
//  delete laser_;
//  laser_ = new amcl::AMCLLaser(max_beams_, map_);
//  ROS_ASSERT(laser_);
//  if(laser_model_type_ == amcl::LASER_MODEL_BEAM)
//    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
//                         sigma_hit_, lambda_short_, 0.0);
//  else if(laser_model_type_ == amcl::LASER_MODEL_LIKELIHOOD_FIELD_PROB){
//    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
//    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
//					laser_likelihood_max_dist_, 
//					do_beamskip_, beam_skip_distance_, 
//					beam_skip_threshold_, beam_skip_error_threshold_);
//    ROS_INFO("Done initializing likelihood field model.");
//  }
//  else
//  {
//    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
//    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
//                                    laser_likelihood_max_dist_);
//    ROS_INFO("Done initializing likelihood field model.");
//  }
//
//  //update the filter
//  // In case the initial pose message arrived before the first map,
//  // try to apply the initial pose now that the map has arrived.
//  applyInitialPose();
//
//}
//
///**
// * Convert an OccupancyGrid map message into the internal
// * representation.  This allocates a map_t and returns it.
// */
//template<class D>
//map_t*
//MCL<D>::convertMap( const nav_msgs::OccupancyGrid& map_msg )
//{
//  map_t* map = map_alloc();
//  ROS_ASSERT(map);
//
//  map->size_x = map_msg.info.width;
//  map->size_y = map_msg.info.height;
//  map->scale = map_msg.info.resolution;
//  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
//  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
//  // Convert to player format
//  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
//  ROS_ASSERT(map->cells);
//  for(int i=0;i<map->size_x * map->size_y;i++)
//  {
//    if(map_msg.data[i] == 0)
//      map->cells[i].occ_state = -1;
//    else if(map_msg.data[i] == 100)
//      map->cells[i].occ_state = +1;
//    else
//      map->cells[i].occ_state = 0;
//  }
//
//  return map;
//}
//
//template<class D>
//void
//MCL<D>::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
//{
//  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
//  if(msg.header.frame_id == "")
//  {
//    // This should be removed at some point
//    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
//  }
//  // We only accept initial pose estimates in the global frame, #5148.
//  else if(tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
//  {
//    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
//             msg.header.frame_id.c_str(),
//             global_frame_id_.c_str());
//    return;
//  }
//
//  // In case the client sent us a pose estimate in the past, integrate the
//  // intervening odometric change.
//  tf::StampedTransform tx_odom;
//  try
//  {
//    ros::Time now = ros::Time::now();
//    // wait a little for the latest tf to become available
//    tf_->waitForTransform(base_frame_id_, msg.header.stamp,
//                         base_frame_id_, now,
//                         odom_frame_id_, ros::Duration(0.5));
//    tf_->lookupTransform(base_frame_id_, msg.header.stamp,
//                         base_frame_id_, now,
//                         odom_frame_id_, tx_odom);
//  }
//  catch(tf::TransformException e)
//  {
//    // If we've never sent a transform, then this is normal, because the
//    // global_frame_id_ frame doesn't exist.  We only care about in-time
//    // transformation for on-the-move pose-setting, so ignoring this
//    // startup condition doesn't really cost us anything.
//    if(sent_first_transform_)
//      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
//    tx_odom.setIdentity();
//  }
//
//  tf::Pose pose_old, pose_new;
//  tf::poseMsgToTF(msg.pose.pose, pose_old);
//  pose_new = pose_old * tx_odom;
//
//  // Transform into the global frame
//
//  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
//           ros::Time::now().toSec(),
//           pose_new.getOrigin().x(),
//           pose_new.getOrigin().y(),
//           MCL::getYaw(pose_new));
//  // Re-initialize the filter
//  //update hypothesis
//  pf_vector_t pf_init_pose_mean = pf_vector_zero();
//  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
//  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
//  pf_init_pose_mean.v[2] = MCL::getYaw(pose_new);
//  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
//  // Copy in the covariance, converting from 6-D to 3-D
//  for(int i=0; i<2; i++)
//  {
//    for(int j=0; j<2; j++)
//    {
//      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
//    }
//  }
//  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];
//
//  delete initial_pose_hyp_;
//  initial_pose_hyp_ = new amcl_hyp_t();
//  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
//  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
//
//  //update the filter
//  applyInitialPose();
//}
//
///**
// * If initial_pose_hyp_ and map_ are both non-null, apply the initial
// * pose to the particle filter state.  initial_pose_hyp_ is deleted
// * and set to NULL after it is used.
// */
//template<class D>
//void
//MCL<D>::applyInitialPose()
//{
//  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
//  if( initial_pose_hyp_ != NULL && map_ != NULL )
//  {
//    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
//    pf_init_ = false;
//
//    delete initial_pose_hyp_;
//    initial_pose_hyp_ = NULL;
//
//    //move build_density_tree to Derived::applyInitialPose()
//    static_cast<D*>(this)->AIP();
//  }
//}
//
//template<class D>
//void 
//MCL<D>::checkLaserReceived(const ros::TimerEvent& event)
//{
//  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
//  if(d > laser_check_interval_)
//  {
//    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
//             d.toSec(),
//             ros::names::resolve(scan_topic_).c_str());
//  }
//}
//
//template<class D>
//void 
//MCL<D>::runFromBag(const std::string &in_bag_fn)
//{
//  rosbag::Bag bag;
//  bag.open(in_bag_fn, rosbag::bagmode::Read);
//  std::vector<std::string> topics;
//  topics.push_back(std::string("tf"));
//  std::string scan_topic_name = "base_scan"; // TODO determine what topic this actually is from ROS
//  topics.push_back(scan_topic_name);
//  rosbag::View view(bag, rosbag::TopicQuery(topics));
//
//  ros::Publisher laser_pub = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_name, 100);
//  ros::Publisher tf_pub = nh_.advertise<tf2_msgs::TFMessage>("/tf", 100);
//
//  // Sleep for a second to let all subscribers connect
//  ros::WallDuration(1.0).sleep();
//
//  ros::WallTime start(ros::WallTime::now());
//
//  // Wait for map
//  while (ros::ok())
//  {
//    {
//      boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
//      if (map_)
//      {
//        ROS_INFO("Map is ready");
//        break;
//      }
//    }
//    ROS_INFO("Waiting for map...");
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0));
//  }
//
//  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
//  {
//    if (!ros::ok())
//    {
//      break;
//    }
//
//    // Process any ros messages or callbacks at this point
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration());
//
//    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
//    if (tf_msg != NULL)
//    {
//      tf_pub.publish(msg);
//      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
//      {
//        tf_->getBuffer().setTransform(tf_msg->transforms[ii], "rosbag_authority");
//      }
//      continue;
//    }
//
//    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
//    if (base_scan != NULL)
//    {
//      laser_pub.publish(msg);
//      laser_scan_filter_->add(base_scan);
//      if (bag_scan_period_ > ros::WallDuration(0))
//      {
//        bag_scan_period_.sleep();
//      }
//      continue;
//    }
//
//    ROS_WARN_STREAM("Unsupported message type" << msg.getTopic());
//  }
//
//  bag.close();
//
//  double runtime = (ros::WallTime::now() - start).toSec();
//  ROS_INFO("Bag complete, took %.1f seconds to process, shutting down", runtime);
//
//  const geometry_msgs::Quaternion & q(this->last_published_pose.pose.pose.orientation);
//  double yaw, pitch, roll;
//  tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
//  ROS_INFO("Final location %.3f, %.3f, %.3f with stamp=%f",
//            last_published_pose.pose.pose.position.x,
//            last_published_pose.pose.pose.position.y,
//            yaw, last_published_pose.header.stamp.toSec()
//            );
//
//  ros::shutdown();
//}
//
//template<class D>
//bool
//MCL<D>::globalLocalizationCallback(std_srvs::Empty::Request& req,
//                                std_srvs::Empty::Response& res)
//{
//  if( map_ == NULL ) {
//    return true;
//  }
//  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
//  ROS_INFO("Initializing with uniform distribution");
//  pf_init_model(pf_, (pf_init_model_fn_t)MCL::uniformPoseGenerator,
//                (void *)map_);
//
//  ROS_INFO("Global initialisation done!");
//  pf_init_ = false;
//
//  //move build_density_tree to Derived::globalLocalizationCB()
//  static_cast<D*>(this)->GLCB();
//  return true;
//}
//
//template<class D>
//void
//MCL<D>::savePoseToServer()
//{
//  // We need to apply the last transform to the latest odom pose to get
//  // the latest map pose to store.  We'll take the covariance from
//  // last_published_pose.
//  tf::Pose map_pose = latest_tf_.inverse() * latest_odom_pose_;
//  double yaw,pitch,roll;
//  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);
//
//  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );
//
//  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
//  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
//  private_nh_.setParam("initial_pose_a", yaw);
//  private_nh_.setParam("initial_cov_xx", 
//                                  last_published_pose.pose.covariance[6*0+0]);
//  private_nh_.setParam("initial_cov_yy", 
//                                  last_published_pose.pose.covariance[6*1+1]);
//  private_nh_.setParam("initial_cov_aa", 
//                                  last_published_pose.pose.covariance[6*5+5]);
//}
//
//template<class D>
//void
//MCL<D>::updatePoseFromServer()
//{
//  init_pose_[0] = 0.0;
//  init_pose_[1] = 0.0;
//  init_pose_[2] = 0.0;
//  init_cov_[0] = 0.5 * 0.5;
//  init_cov_[1] = 0.5 * 0.5;
//  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
//  // Check for NAN on input from param server, #5239
//  double tmp_pos;
//  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
//  if(!std::isnan(tmp_pos))
//    init_pose_[0] = tmp_pos;
//  else 
//    ROS_WARN("ignoring NAN in initial pose X position");
//  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
//  if(!std::isnan(tmp_pos))
//    init_pose_[1] = tmp_pos;
//  else
//    ROS_WARN("ignoring NAN in initial pose Y position");
//  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
//  if(!std::isnan(tmp_pos))
//    init_pose_[2] = tmp_pos;
//  else
//    ROS_WARN("ignoring NAN in initial pose Yaw");
//  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
//  if(!std::isnan(tmp_pos))
//    init_cov_[0] =tmp_pos;
//  else
//    ROS_WARN("ignoring NAN in initial covariance XX");
//  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
//  if(!std::isnan(tmp_pos))
//    init_cov_[1] = tmp_pos;
//  else
//    ROS_WARN("ignoring NAN in initial covariance YY");
//  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
//  if(!std::isnan(tmp_pos))
//    init_cov_[2] = tmp_pos;
//  else
//    ROS_WARN("ignoring NAN in initial covariance AA");	
//}
//
//template<class D>
//void
//MCL<D>::freeMapDependentMemory()
//{
//  if( map_ != NULL ) {
//    map_free( map_ );
//    map_ = NULL;
//  }
//  if( pf_ != NULL ) {
//    pf_free( pf_ );
//    pf_ = NULL;
//  }
//  delete odom_;
//  odom_ = NULL;
//  delete laser_;
//  laser_ = NULL;
//}
//
//template<class D>
//MCL<D>::~MCL()
//{
//  delete dsrv_;
//  freeMapDependentMemory();
//  //delete laser_scan_filter_;
//  delete laser_scan_sub_;
//  delete tfb_;
//  delete tf_;
//}
//
//template<class D>
//bool
//MCL<D>::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
//                 double& x, double& y, double& yaw,
//                 const ros::Time& t, const std::string& f)
//{
//  // Get the robot's pose
//  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
//                                           tf::Vector3(0,0,0)), t, f);
//  try
//  {
//    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
//  }
//  catch(tf::TransformException e)
//  {
//    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
//    return false;
//  }
//  x = odom_pose.getOrigin().x();
//  y = odom_pose.getOrigin().y();
//  double pitch,roll;
//  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
//
//  return true;
//}
//
//template<class D>
//pf_vector_t
//MCL<D>::uniformPoseGenerator(void* arg)
//{
//  map_t* map = (map_t*)arg;
//#if NEW_UNIFORM_SAMPLING
//  //unsigned int rand_index = drand48() * MCL::free_space_indices.size();
//  unsigned int rand_index = MCL::rng_.uniform01() * MCL::free_space_indices.size();
//  std::pair<int,int> free_point = MCL::free_space_indices[rand_index];
//  pf_vector_t p;
//  p.v[0] = MAP_WXGX(map, free_point.first);
//  p.v[1] = MAP_WYGY(map, free_point.second);
//  //p.v[2] = drand48() * 2 * M_PI - M_PI;
//  p.v[2] = MCL::rng_.uniform01() * 2 * M_PI - M_PI;
//#else
//  double min_x, max_x, min_y, max_y;
//
//  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
//  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
//  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
//  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;
//
//  pf_vector_t p;
//
//  ROS_DEBUG("Generating new uniform sample");
//  for(;;)
//  {
//    /*p.v[0] = min_x + drand48() * (max_x - min_x);
//    p.v[1] = min_y + drand48() * (max_y - min_y);
//    p.v[2] = drand48() * 2 * M_PI - M_PI;*/
//    p.v[0] = min_x + MCL::rng_.uniform01() * (max_x - min_x);
//    p.v[1] = min_y + MCL::rng_.uniform01() * (max_y - min_y);
//    p.v[2] = MCL::rng_.uniform01() * 2 * M_PI - M_PI;
//    // Check that it's a free cell
//    int i,j;
//    i = MAP_GXWX(map, p.v[0]);
//    j = MAP_GYWY(map, p.v[1]);
//    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
//      break;
//  }
//#endif
//  return p;
//}
//
template<class D>
double
MCL<D>::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

#endif//MCL_H
