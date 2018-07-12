#ifndef DEMC_H
#define DEMC_H
#include <map>
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include "random_numbers/random_numbers.h"
#include <nuklei/KernelCollection.h>
#include "amcl/pf/pf.h"

namespace demc{
/**
 * @brief The parameter struct of DEMC algorithm
 * @details gamma is the step size, loc_bw is the bandwidth for position values, and ori_bw is the bandwidth for orientation
 */
typedef struct 
{
  double gamma;
  double loc_bw;
  double ori_bw;
} demc_t;

/**
 * @brief This function implements MCMC jump of DEMC algorithm.
 *
 * @param pool This gene pool of DEMC is the particles distributed over p(x_t|u_t,x_{t-1})
 * @param[in] params This parameters of DEMC algorithm
 * @param[in] mapx The pair of minimum and maximum values in X-axis of map coordinate in meters
 * @param[in] mapy The pair of minimum and maximum values in Y-axis of map coordinate in meters
 * @param[in] mapx_range The distance between the minimum and maximum in X-axis in meters
 * @param[in] mapy_range The distance between the minimum and maximum in Y-axis in meters
 * @param[in] rng The generator of random values
 * @param[out] pop The output population of DEMC algorithm
 */
void proposal(
  pf_sample_set_t* pool, 
  demc_t* params,
  std::pair<double, double> mapx,
  std::pair<double, double> mapy,
  double mapx_range,
  double mapy_range,
  random_numbers::RandomNumberGenerator rng,
  pf_sample_set_t* pop)
{
  pop->sample_count = pool->sample_count;
  pf_sample_t* child;
  pf_sample_t* parent;
  pf_sample_t* rand1;
  pf_sample_t* rand2;
  int r1, r2;
  pf_vector_t diff;
  double tmp;
  for(int i = 0 ; i < pop->sample_count ; ++i)
  {
    child = pop->samples + i;
    parent = pool->samples + i;
    r1 = rng.uniformInteger(0, pool->sample_count-1);
    r2 = rng.uniformInteger(0, pool->sample_count-1);
    while(r2==r1)
      r2 = rng.uniformInteger(0, pool->sample_count-1);
    rand1 = pool->samples + r1;
    rand2 = pool->samples + r2;
    diff = pf_vector_sub(rand1->pose, rand2->pose);
    diff.v[0] = params->gamma * diff.v[0] + rng.gaussian(0, params->loc_bw);
    diff.v[1] = params->gamma * diff.v[1] + rng.gaussian(0, params->loc_bw);
    diff.v[2] = params->gamma * angle_diff( (rand1->pose.v[2]), (rand2->pose.v[2]) ) + rng.gaussian(0, params->ori_bw);
    child->pose = pf_vector_add(parent->pose, diff);
    if( child->pose.v[0] > mapx.second || child->pose.v[0] < mapx.first)
      child->pose.v[0] = fmod( fmod( child->pose.v[0] - mapx.first , mapx_range) + mapx_range, mapx_range) + mapx.first;
    if( child->pose.v[1] > mapy.second || child->pose.v[1] < mapy.first)
      child->pose.v[1] = fmod( fmod( child->pose.v[1] - mapy.first , mapy_range) + mapy_range, mapy_range) + mapy.first;
    child->pose.v[2] = normalize(child->pose.v[2]);
    //set all weights to be one
    child->weight = 1.0;
  }
}

/**
 * @brief This function implements Metropolis algorithm and weight mixing method of Mixture-MCL
 * The particles accepted by Metropolis are seen as the samples drawn from measurement model.
 * Then it uses kernel density estimation to estimate the density probability of those particles.
 *
 * @param[in] ldata The object for measurement model
 * @param[in] ita The normalizer for Mixture-MCL
 * @param[in] kdt The object for Kernel Density Estimation
 * @param[in] demc_params The parameters for DEMC algorithm, a version of Metropolis algorithm
 * @param[in] mapx  
 * @param[in] mapy
 * @param[in] mapx_range
 * @param[in] mapy_range
 * @param[in] rng Random number generator
 * @param[in] old_chains Markov chains at current iteration
 * @param[out] new_chains Markov chains at next iteration
 * @param[out] accepted_cloud Pose array for publishing to topics
 * @param[out] rejected_cloud Pose array for publishing to topics
 * @return Total weight of all evaluated particles
 */
double metropolisRejectAndCalculateWeight(
  amcl::AMCLLaserData& ldata, 
  double ita,
  nuklei::KernelCollection* kdt,
  demc_t* demc_params,
  std::pair<double, double> mapx,
  std::pair<double, double> mapy,
  double mapx_range,
  double mapy_range,
  random_numbers::RandomNumberGenerator rng,
  pf_sample_set_t* old_chains, //source particles with weight
  pf_sample_set_t* new_chains, //sampled particles with weight
  geometry_msgs::PoseArray& accepted_cloud,
  geometry_msgs::PoseArray& rejected_cloud)
{
  //propose states of new chains
  demc::proposal(old_chains, demc_params, mapx, mapy, mapx_range, mapy_range, rng, new_chains);
  //update_measurement_model for both old_chains and new_chains
  //note that old_chains is resampled particle set with equal weights
  ((amcl::AMCLLaser*)ldata.sensor)->UpdateSensorWithSet(old_chains, &ldata);
  ((amcl::AMCLLaser*)ldata.sensor)->UpdateSensorWithSet(new_chains, &ldata);
  
  //for each sample of new_chains, apply acceptance and rejection scheme
  double total = 0;
  double log_uniform, log_alpha;
  pf_sample_t* old_state;
  pf_sample_t* new_state;
  pf_vector_t vec_pose;
  nuklei::kernel::se3 se3_pose;
  for(int i = 0 ; i < new_chains->sample_count ; ++i)
  {
    old_state = old_chains->samples + i;
    new_state = new_chains->samples + i;
    //calculate alpha
    log_alpha = new_state->logWeight - old_state->logWeight;
    if(log_alpha < 0)
      log_uniform = std::log(rng.uniform01());
    else
      log_uniform = 0;

    //if accepted 
    if(log_uniform <= log_alpha)
    {
      //calculate weight for new_state according to kernel density tree of previous poses
      //move the sample from new_chains to old_chains
      old_state->pose = new_state->pose;
      //vec_pose = new_state->pose;
      //nuklei::kernel::se3 se3_pose;
      MixmclNode::poseToSe3(old_state->pose, se3_pose);
      old_state->weight = ita * (kdt->evaluationAt(se3_pose));

      //publish the accepted particle to particlecloud2
      geometry_msgs::Pose p;
      tf::poseTFToMsg(
        tf::Pose(
          tf::createQuaternionFromYaw(
            old_state->pose.v[2]),
          tf::Vector3(
            old_state->pose.v[0],
            old_state->pose.v[1], 
            0)),
          p);
      accepted_cloud.poses.push_back(p);
    }
    //if rejected
    else
    {
      //publishe the rejected ones to particlecloud3 topic
      geometry_msgs::Pose p;
      tf::poseTFToMsg(
        tf::Pose(
          tf::createQuaternionFromYaw(
            new_state->pose.v[2]),
          tf::Vector3(
            new_state->pose.v[0],
            new_state->pose.v[1], 
            0)),
          p);
      rejected_cloud.poses.push_back(p);
    }
    
    total += old_state->weight;
  }
  return total;
}

} // namespace
#endif //DEMC_H
