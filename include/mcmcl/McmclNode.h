#ifndef MCMCLNODE_H
#define MCMCLNODE_H
#include "mcl/MCL.h"
#include "mixmcl/MixmclNode.h"
#include "mixmcl/MCMCLConfig.h"
//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>
#include "demc.h"

//static inline double gausspdf(double a, double b)
//{
//  double a2 = a*a;
//  double b2 = b*b;
//  return exp(a2/(b2*(-2)))/(b*sqrt(2*M_PI));
//}

//TODO inherit amcl::odom_model_t and expand it with following parameters
//typedef struct
//{
//  double delta_rot1;
//  double delta_trans;
//  double delta_rot2;
//} odom_model_t;

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

    //pf_sample_ptr_vector_t set_c;  
    //TODO mixmcl parameters 
    double ita_;
    double loch_, orih_;
    boost::shared_ptr<demc::demc_t> demc_params_;
    boost::shared_ptr<nuklei::KernelCollection> kdt_;
    ros::Publisher particlecloud2_pub_;//for accepted cloud
    ros::Publisher particlecloud3_pub_;//for rejected cloud
    bool first_reconfigureCB2_call_;
    bool version1_;
    bool static_update_;
    dynamic_reconfigure::Server<mixmcl::MCMCLConfig> *dsrv2_;
    mixmcl::MCMCLConfig default_config2_;
    void reconfigureCB2(mixmcl::MCMCLConfig& config, uint32_t level);
    void printInfo()
    {
      ROS_INFO("ita: %f", ita_);
      ROS_INFO("loch: %f", loch_);
      ROS_INFO("orih: %f", orih_);
    };
};

#endif//MCMCLNODE_H
