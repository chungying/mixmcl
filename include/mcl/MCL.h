#ifndef MCL_H
#define MCL_H
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "amcl_modified/map/map.h"
#include "amcl_modified/pf/pf.h"
#include "amcl_modified/sensors/amcl_odom.h"
#include "amcl_modified/sensors/amcl_laser.h"
#include "amcl/pf/pf_resample.h"

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
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "stamped_std_msgs/StampedFloat64MultiArray.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl_modified/AMCLConfig.h"

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

    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif

    static inline double getYaw(tf::Pose& t);

    static void publishParticleCloud( ros::Publisher& particlecloud_pub_, const std::string& global_frame_id_, const ros::Time& stamp, pf_t* pf_, int set_drift);

    static void publishWeightedParticleCloud( ros::Publisher& wpc_pub_, const std::string& global_frame_id_, const ros::Time& stamp, pf_t* pf_, int set_drift);

    void createLaserData(int laser_index, amcl::AMCLLaserData& ldata, const sensor_msgs::LaserScanConstPtr& laser_scan);

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

    bool global_localization_;
    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    //for map information
    map_t* map_;
    std::pair<double, double> mapx_;
    std::pair<double, double> mapy_;
    double map_rng_x_;
    double map_rng_y_;
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
    void (*resample_function_)(pf_t* );
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
    ros::Publisher wpc_pub_;//weighted particle cloud;
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
    void printInfo()
    {
      ROS_INFO("min_particles: %d", min_particles_);
      ROS_INFO("max_particles: %d", max_particles_);
      ROS_INFO("kld_err_: %f", pf_err_);
      ROS_INFO("kld_z_: %f", pf_z_);
      ROS_INFO("update_min_d: %f", d_thresh_);
      ROS_INFO("update_min_a: %f", a_thresh_);
      ROS_INFO("resample_interval_: %d", resample_interval_);
      ROS_INFO("transform_tolerance: %f secs", transform_tolerance_.toSec());
      ROS_INFO("recovery_alpha_slow: %f", alpha_slow_);
      ROS_INFO("recovery_alpha_fast: %f", alpha_fast_);
      ROS_INFO("save_pose_rate: %f secs", save_pose_period.toSec());
      ROS_INFO("laser_max_beams: %d", max_beams_);
      ROS_INFO("laser_min_range_: %f", laser_min_range_);
      ROS_INFO("laser_max_range_: %f", laser_max_range_);
      ROS_INFO("z: %f %f %f %f", z_hit_, z_short_, z_max_, z_rand_);
      ROS_INFO("sigma lamda: %f %f", sigma_hit_, lambda_short_);
      ROS_INFO("alphas: %f %f %f %f %f", alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
      ROS_INFO("max: [%f %f], min: [%f %f], range: [%f %f]", mapx_.second, mapy_.second, mapx_.first, mapy_.first, map_rng_x_, map_rng_y_);
    };
};

template<class D>
std::vector<std::pair<int,int> > MCL<D>::free_space_indices;
template<class D>
random_numbers::RandomNumberGenerator MCL<D>::rng_;

template<class D>
double
MCL<D>::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

template<class D>
void MCL<D>::publishWeightedParticleCloud(
  ros::Publisher& wpc_pub_,
  const std::string& global_frame_id,
  const ros::Time& stamp,
  pf_t* pf_,
  int set_shift = 0)
{
  // Publish the resulting cloud
  pf_sample_set_t* set = pf_->sets + ((pf_->current_set + set_shift) % 2);
  ROS_INFO("publishing %d weighted particles",set->sample_count);
  stamped_std_msgs::StampedFloat64MultiArray wpc_msg;
  wpc_msg.header.frame_id = global_frame_id;
  wpc_msg.header.stamp = stamp;
  wpc_msg.array.layout.dim.resize(2);
  wpc_msg.array.layout.dim[0].label = "particle";
  wpc_msg.array.layout.dim[0].size = set->sample_count;
  wpc_msg.array.layout.dim[0].stride = set->sample_count * 4;
  wpc_msg.array.layout.dim[1].label = "xyaw";
  wpc_msg.array.layout.dim[1].size = 4;
  wpc_msg.array.layout.dim[1].stride = 4;
  wpc_msg.array.data.resize(set->sample_count);
  for(int idx=0; idx < set->sample_count;++idx)
  {
    wpc_msg.array.data[idx*4  ] = set->samples[idx].pose.v[0];
    wpc_msg.array.data[idx*4+1] = set->samples[idx].pose.v[1];
    wpc_msg.array.data[idx*4+2] = set->samples[idx].pose.v[2];
    wpc_msg.array.data[idx*4+3] = set->samples[idx].weight;
  }
  wpc_pub_.publish(wpc_msg);
}

template<class D>
void MCL<D>::publishParticleCloud(
  ros::Publisher& particlecloud_pub_,
  const std::string& global_frame_id_,
  const ros::Time& stamp,
  pf_t* pf_,
  int set_shift = 0)
{
  // Publish the resulting cloud
  pf_sample_set_t* set = pf_->sets + ((pf_->current_set + set_shift) % 2);
  ROS_INFO("publishing %d particles",set->sample_count);
  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  for(int i=0;i<set->sample_count;i++)
  {
    tf::poseTFToMsg(
      tf::Pose(
        tf::createQuaternionFromYaw(
          set->samples[i].pose.v[2]),
        tf::Vector3(
          set->samples[i].pose.v[0],
          set->samples[i].pose.v[1], 
          0)),
      cloud_msg.poses[i]);
  }
  particlecloud_pub_.publish(cloud_msg);
}

#endif//MCL_H
