#include "mcl/MCL.h"
#include "stamped_std_msgs/StampedFloat64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include <thread>
#include <mutex>
#include <functional>
//convert from angle to index
#define ANG2IDX(ang, ares) (floor(((ang + M_PI)/M_PI)*(180.0/ares)+0.5))
//convert from index to angle
#define IDX2ANG(idx, ares) (-M_PI+((idx*ares)/180.0)*M_PI)

//Markov Grid Loclization
class MarkovNode : public MCL<MarkovNode> 
{
  //types
  typedef std::vector<double> Matrix;
  typedef std::vector<Matrix> VecMatrices;
  typedef std::vector<VecMatrices > MatMatrices;
  typedef MatMatrices::iterator mIter;
  typedef std::vector<double>::iterator aIter;

  friend class MCL;
  public:
    MarkovNode();
    ~MarkovNode();
  protected:
    ros::Publisher histograms_pub_;
    std_msgs::MultiArrayLayout hist_layout_;
    ros::Publisher positions_pub_;
    std_msgs::Float64MultiArray positions_msg_;
    ros::Publisher indices_pub_;
    std_msgs::UInt16MultiArray free_idcs_msg_;
    std::vector<std::vector<int> > mapidx2freeidx_;
    pf_t *grid_;
    int cloud_size_;
    int ares_;
    int size_a_;
    double radius_;
    double epson_;
    int laser_buffer_size_;
    std::vector<int> active_sample_indices_;
    void initialMarkovGrid();
    int downsizingSampling(pf_sample_set_t* set_a, pf_sample_set_t* set_b, int target_size);

    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    double UpdateOdom(amcl::AMCLOdomData* ndata);
    double motionModelS(const pf_sample_t* sample_a, const pf_sample_t* sample_b, const amcl::AMCLOdom* odom, const double delta_rot1, const double delta_trans, const double delta_rot2);
    double UpdateOdomO(amcl::AMCLOdomData* ndata);
    static void odometry(const double oldx, const double oldy, const double olda, const double newx, const double newy, const double newa, double& delta_rot1_hat, double& delta_trans_hat, double& delta_rot2_hat);
    static double motionModelO(const amcl::AMCLOdom* odom, const double delta_rot1, const double delta_trans, const double delta_rot2, const double delta_rot1_hat, const double delta_trans_hat, const double delta_rot2_hat);
    void GLCB(){};
    void AIP(){};
    void RCCB()
    {
      delete laser_scan_filter_;
      laser_scan_filter_ = 
              new tf::MessageFilter<sensor_msgs::LaserScan>(
                        *laser_scan_sub_, 
                        *tf_, 
                        odom_frame_id_, 
                        100);
      laser_scan_filter_->registerCallback(
                  boost::bind(&MarkovNode::laserReceived,
                  this, _1));
    };

};

