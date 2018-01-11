#ifndef MCMCLNODE_H
#define MCMCLNODE_H
#include "mcl/MCL.h"
#include "mixmcl/MixmclNode.h"
#include "mixmcl/MCMCLConfig.h"
//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>

static inline double gausspdf(double a, double b)
{
  double a2 = a*a;
  double b2 = b*b;
  return exp(a2/(b2*(-2)))/(b*sqrt(2*M_PI));
}

typedef struct
{
  double delta_rot1;
  double delta_trans;
  double delta_rot2;
} odom_model_t;

class McmclNode : public MCL<McmclNode> 
{
  friend class MCL;
  public:
    typedef std::vector<boost::shared_ptr<pf_sample_t> > pf_sample_ptr_vector_t;  
    McmclNode();
    ~McmclNode();

  protected:
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void GLCB()
    {
      ROS_INFO("McmclNode::GLCB() is called. Build density tree..");
      MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    void AIP()
    {
      ROS_INFO("McmclNode::AIP() is called. Build density tree..");
      MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    void RCCB();
   
    static void clone(pf_sample_set_t* src, pf_sample_ptr_vector_t& dst);
    static double evaluateMotionModel(const pf_vector_t& cur_p, const pf_vector_t& pre_p, const odom_model_t& omodel, amcl::AMCLOdom* odom_);

    void demcProposal(pf_sample_set_t* gene_pool, pf_sample_set_t* population);

    pf_sample_ptr_vector_t set_c;  
    double ita_;
    double gamma_;
    double loc_bw_, ori_bw_;
    double loch_, orih_;
    boost::shared_ptr<nuklei::KernelCollection> kdt_;
    ros::Publisher particlecloud2_pub_;
    bool first_reconfigureCB2_call_;
    dynamic_reconfigure::Server<mixmcl::MCMCLConfig> *dsrv2_;
    mixmcl::MCMCLConfig default_config2_;
    void reconfigureCB2(mixmcl::MCMCLConfig& config, uint32_t level);
};

#endif//MCMCLNODE_H
