#include "mcl/MCL.h"
#include "mixmcl/laser_feature.h"
#include "mixmcl/KCGrid.h"
// Dynamic_reconfigure
#include "mixmcl/MIXMCLConfig.h"

//Allows MCL to build density trees for particle evaluation
#include <nuklei/KernelCollection.h>
class MixmclNode : public MCL<MixmclNode>
{
  friend class MCL;
  public:
    MixmclNode();
    ~MixmclNode();
    static void buildDensityTree(pf_t* pf, boost::shared_ptr<nuklei::KernelCollection>& kdt, double loch, double orih);//build a KernelCollection based on previous weighted set for evaluating current dual set
    static inline void poseToSe3(const pf_vector_t& vec_p, nuklei::kernel::se3& se3_p);
    static inline void se3ToPose(const nuklei::kernel::se3& se3_p, pf_vector_t& vec_p);
  protected:
    //inheritance
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void GLCB()
    {
      //ROS_INFO("MixmclNode::GLCB() is called. Build density tree..");
      //buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    void AIP()
    {
      //ROS_INFO("MixmclNode::AIP() is called. Build density tree..");
      //buildDensityTree(pf_, kdt_, loch_, orih_);
    };
    virtual void RCCB();

    //Mixture MCL
    //TODO wrap Mixture-MCL paprameters together as a struct or class
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
    void mixtureProposals();//determin the size of dual set and regular set
    double dualmclNEvaluation( amcl::AMCLLaserData& ldata, amcl::AMCLOdomData& inverse_odata);
    void createKCGrid();//read data from binary file and create a discrete KernelCollection grid
    void reconfigureCB2(mixmcl::MIXMCLConfig &config, uint32_t level);
    void printInfo()
    {
      ROS_INFO("loch: %f", loch_);
      ROS_INFO("orih: %f", orih_);
      ROS_INFO("mixing_rate: %f", mixing_rate_);
      ROS_INFO("ita: %f", ita_);
      ROS_INFO("fxres, fyres, fdres: %d %d %d", fxres_, fyres_, fdres_);
      ROS_INFO("param_filename: %s", sample_param_filename_.c_str());
    };
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
  double ax, ay, az;
  mat.ExtractEulerZYX(az, ay, ax);
  vec_p.v[2] = az;
}

