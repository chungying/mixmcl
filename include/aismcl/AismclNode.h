/**
 * @file AismclNode.h
 * @brief AismclNode implements Monte Carlo Localization algorithm with combination of Annealed Importance Sampling
 * @author chungying
 * @version 0.0.0
 * @date 2018-05-25
 */
#ifndef MCMCLNODE_H
#define MCMCLNODE_H
#include "mcl/MCL.h"
#include "mixmcl/MixmclNode.h"
#include "mixmcl/MCMCLConfig.h"
//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>
#include "demc.h"

namespace ais
{

enum density_t
{
  uniform = 0,
  logarithm
};

typedef struct 
{
  //the number of MCMC iteration
  int iter_num;
  //the type of bridging densities 
  density_t den_type;

} ais_t;

}


class AismclNode : public MCL<AismclNode> 
{
  friend class MCL;
  public:
    typedef std::vector<boost::shared_ptr<pf_sample_t> > pf_sample_ptr_vector_t;  
    AismclNode();
    ~AismclNode();

  protected:
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void GLCB()
    {
      ROS_INFO("AismclNode::GLCB() is called. Build density tree..");
      MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    void AIP()
    {
      ROS_INFO("AismclNode::AIP() is called. Build density tree..");
      MixmclNode::buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    void RCCB();

    //pf_sample_ptr_vector_t set_c;  
    //TODO wrapping mixmcl parameters 
    double ita_;
    double loch_, orih_;
    boost::shared_ptr<demc::demc_t> demc_params_;
    boost::shared_ptr<ais::ais_t> ais_params_;
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
