#ifndef SAMPLING_H
#define SAMPLING_H

#include <fstream>
#include "gazebo_msgs/SetModelState.h"
#include "mixmcl/mixmcl.h"
#include "mixmcl/laser_feature.h"
#include "io/dataio.h"
#include "io/paramio.h"

//class SamplingNode : public MixmclNode
class SamplingNode : public MCL<SamplingNode>
{
  friend class MCL;
  public:
    SamplingNode();
    ~SamplingNode();
    void moveRobotUniformly();
    

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

    int max_data_count;
    //for param.txt
    int dataCount;
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
    void convertKCGrid(){};
    bool initFlag;
    bool randomSucceed;
    bool laserUpdated;
    ros::ServiceClient cli_;// SetModelState client
    std::string srv_name;
    gazebo_msgs::ModelState state;
    laser_feature_t curFeature;
    pf_vector_t pose;    
    std::string modelName;
    std::string refFrame;

};

const static std::string fp = "/fullpath";
const static std::string ts = "/timestamp";


#endif //SAMPLING_H
