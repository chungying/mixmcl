#ifndef AISMCLNODE_H
#define AISMCLNODE_H
#include "mcl/MCL.h"
#include "mixmcl/MCMCLConfig.h"
//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>

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
   
    //TODO make these two function static
    double metropolisNEvaluation(geometry_msgs::PoseArray& accepted_cloud, geometry_msgs::PoseArray& rejected_cloud, amcl::AMCLLaserData& ldata, double ita);
    void demcProposal(pf_sample_set_t* gene_pool, pf_sample_set_t* population);

    //pf_sample_ptr_vector_t set_c;  
    //TODO MCMC sampler struct
    double ita_;
    double gamma_;
    double loc_bw_, ori_bw_;
    double loch_, orih_;

    //TODO nuklei struct
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
      ROS_INFO("gamma: %f", gamma_);
      ROS_INFO("loc_bw: %f", loc_bw_);
      ROS_INFO("ori_bw: %f", ori_bw_);
      ROS_INFO("loch: %f", loch_);
      ROS_INFO("orih: %f", orih_);
    };
};

#endif//AISMCLNODE_H
