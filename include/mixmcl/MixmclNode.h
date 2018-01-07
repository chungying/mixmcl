#include "mcl/MCL.h"
#include "mixmcl/laser_feature.h"
#include "mixmcl/KCGrid.h"

// Dynamic_reconfigure
#include "mixmcl/MIXMCLConfig.h"

//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>
#include <nuklei/Kernel.h>
class MixmclNode : public MCL<MixmclNode>
{
  friend class MCL;
  public:
    MixmclNode();
    ~MixmclNode();
  protected:
    //inheritance
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void GLCB()
    {
      ROS_INFO("MixmclNode::GLCB() is called. Build density tree..");
      build_density_tree();
    };
    void AIP()
    {
      ROS_INFO("MixmclNode::AIP() is called. Build density tree..");
      build_density_tree();
    };
    virtual void RCCB();

    //Mixture MCL
    double mixing_rate_, ita_;
    int fxres_, fyres_, fdres_;
    std::string sample_param_filename_;
    boost::shared_ptr<KCGrid> kcgrid_;
    double loch_, orih_;
    boost::shared_ptr<nuklei::KernelCollection> kdt_;
    bool first_reconfigureCB2_call_;
    ros::Publisher particlecloud2_pub_;
    dynamic_reconfigure::Server<mixmcl::MIXMCLConfig> *dsrv2_;
    mixmcl::MIXMCLConfig default_config2_;
    void build_density_tree();
    void mixtureProposals();
    void combineSets();
    void publishCloud2();
    void convertKCGrid();
    void reconfigureCB2(mixmcl::MIXMCLConfig &config, uint32_t level);
    static inline void se3ToPose(const nuklei::kernel::se3& se3_p, pf_vector_t& p);
    static inline void poseToSe3(const pf_vector_t& p, nuklei::kernel::se3& se3_p);

};

inline void
MixmclNode::poseToSe3(const pf_vector_t& vec_p, nuklei::kernel::se3& se3_p)
{
  se3_p.loc_.X() = vec_p.v[0];
  se3_p.loc_.Y() = vec_p.v[1];
  tf::Quaternion q = tf::createQuaternionFromYaw(vec_p.v[2]);
  se3_p.ori_.W() = q.w();
  se3_p.ori_.X() = q.x();
  se3_p.ori_.Y() = q.y();
  se3_p.ori_.Z() = q.z();
}

inline void
MixmclNode::se3ToPose(const nuklei::kernel::se3& se3_p, pf_vector_t& vec_p)
{
  vec_p.v[0] = se3_p.loc_.X();
  vec_p.v[1] = se3_p.loc_.Y();
  nuklei::Matrix3 mat;
  se3_p.ori_.ToRotationMatrix(mat);
  nuklei::coord_t ax, ay, az;
  //TODO check the order ZYX
  mat.ExtractEulerZYX(az, ay, ax);
  vec_p.v[2] = (double)az;
}

