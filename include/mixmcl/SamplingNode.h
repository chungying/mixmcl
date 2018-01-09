#ifndef SAMPLING_H
#define SAMPLING_H

#include <fstream>
#include "gazebo_msgs/SetModelState.h"
#include "mixmcl/MixmclNode.h"
#include "mixmcl/laser_feature.h"
#include "io/dataio.h"
#include "io/paramio.h"

class SamplingNode : public MCL<SamplingNode>
{
  friend class MCL;
  public:
    SamplingNode();
    ~SamplingNode();
    void sampling();
    static void raycasting(
      amcl::AMCLLaser* self,
      const pf_vector_t rpose, 
      const int range_count,
      const double range_max,
      const double range_min,
      const double angle_min,
      const double angle_increment,
      amcl::AMCLLaserData& ldata);

  protected:
    //implement virtual functions
    //update laser andd calculate feature
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void GLCB(){};
    void AIP(){};
    void RCCB();


    //save parameters and sampling data
    std::string output_dir_;
    std::string output_prefix_;
    std::string output_filename_data_;
    std::string output_filename_param_;
    boost::scoped_ptr<dataio::DataOut> dataout_ptr_;
    boost::scoped_ptr<paramio::ParamOut> paramout_ptr_;

    //for param.txt
    int data_count_;
    int lrnum;//number of laser beams 
    double noise;//laser_noise got from ParamServer
    double lrmin;//minimum laser range
    double lrmax;//maximum laser range
    double lares;//laser angle resolution
    double lamin;//minimum laser angle
    double lamax;//maximum laser angle
    double fxmin;//minimum x laser feature
    double fxmax;//maximum x laser feature
    double fymin;//minimum y laser feature
    double fymax;//maximum y laser feature
    double fdmin;//minimum distance laser feature
    double fdmax;//maximum distance laser feature

    int max_data_count_;
    bool laserUpdated;
};

const static std::string fp = "/fullpath";
const static std::string ts = "/timestamp";

#endif //SAMPLING_H
