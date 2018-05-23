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
 * @param params This parameters of DEMC algorithm
 * @param mapx The pair of minimum and maximum values in X-axis of map coordinate in meters
 * @param mapy The pair of minimum and maximum values in Y-axis of map coordinate in meters
 * @param mapx_range The distance between the minimum and maximum in X-axis in meters
 * @param mapy_range The distance between the minimum and maximum in Y-axis in meters
 * @param rng The generator of random values
 * @param pop The output population of DEMC algorithm
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
 * @param[in, out] pf The pointer having particle sets
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
  pf_t* pf,
  geometry_msgs::PoseArray& accepted_cloud,
  geometry_msgs::PoseArray& rejected_cloud)
{
  const int set_a_idx = pf->current_set;
  const int set_b_idx = (pf->current_set + 1 ) % 2;
  pf_sample_set_t* set_a = pf->sets + set_a_idx;
  pf_sample_set_t* set_b = pf->sets + set_b_idx;
  demc::proposal(set_a, demc_params, mapx, mapy, mapx_range, mapy_range, rng, set_b);
  //update_measurement_model for both set_a and set_b
  //note that set_a is resampled particle set with equal weights
  pf->current_set = set_a_idx;
  ldata.sensor->UpdateSensor(pf, (amcl::AMCLSensorData*)&ldata);
  pf->current_set = set_b_idx;
  ldata.sensor->UpdateSensor(pf, (amcl::AMCLSensorData*)&ldata);
  pf->current_set = set_a_idx;
  //for each sample of set_b
  double total = 0;
  double log_uniform, log_alpha;
  pf_sample_t* sample_a;
  pf_sample_t* sample_b;
  pf_vector_t vec_pose;
  nuklei::kernel::se3 se3_pose;
  for(int i = 0 ; i < set_b->sample_count ; ++i)
  {
    sample_a = set_a->samples + i;
    sample_b = set_b->samples + i;
    //calculate alpha
    log_alpha = sample_b->logWeight - sample_a->logWeight;
    if(log_alpha < 0)
      log_uniform = std::log(rng.uniform01());
    else
      log_uniform = 0;

    //if accepted 
    if(log_uniform <= log_alpha)
    {
      //calculate weight for sample_b according to kernel density tree of previous poses
      //move the sample from set_b to set_a
      sample_a->pose = sample_b->pose;
      //vec_pose = sample_b->pose;
      //nuklei::kernel::se3 se3_pose;
      MixmclNode::poseToSe3(sample_a->pose, se3_pose);
      sample_a->weight = ita * (kdt->evaluationAt(se3_pose));
      //publish the accepted particle to particlecloud2
      geometry_msgs::Pose p;
      tf::poseTFToMsg(
        tf::Pose(
          tf::createQuaternionFromYaw(
            sample_a->pose.v[2]),
          tf::Vector3(
            sample_a->pose.v[0],
            sample_a->pose.v[1], 
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
            sample_b->pose.v[2]),
          tf::Vector3(
            sample_b->pose.v[0],
            sample_b->pose.v[1], 
            0)),
          p);
      rejected_cloud.poses.push_back(p);
    }
    
    total += sample_a->weight;
  }
  return total;
}

/**
 * @brief This function performs Annealed Importance Sampling incorporating with Metropolis-Hastings sampling methods.
 *
 * @param[in] ldata The object for measurement model
 * @param[in] iteration_number The number of iteration of MH method
 * @param[in] kdt The object for Kernel Density Estimation, if NULL, kdt is uniform distribution
 * @param[in] demc The object for DEMC algorithm of MH method
 * @param[in,out] pf The object for particle sets
 * @return 
 */
double annealedImportanceSampling(
  amcl::AMCLLaserData& ldata, 
  int iter_no,
  nuklei::KernelCollection* kdt,
  demc_t* demc_params,
  std::pair<double, double> mapx,
  std::pair<double, double> mapy,
  double mapx_range,
  double mapy_range,
  random_numbers::RandomNumberGenerator rng,
  pf_t* pf)//,
  //geometry_msgs::PoseArray& accepted_cloud,
  //geometry_msgs::PoseArray& rejected_cloud)
{
  double total = 0.0;
  double log_uniform, log_alpha;
  pf_sample_t* sample_a;
  pf_sample_t* sample_b;
  pf_vector_t vec_pose;
  nuklei::kernel::se3 se3_pose;
  const int set_a_idx = pf->current_set;
  const int set_b_idx = (pf->current_set + 1 ) % 2;
  pf_sample_set_t* set_a = pf->sets + set_a_idx;
  pf_sample_set_t* set_b = pf->sets + set_b_idx;
  set_b->sample_count = set_a->sample_count
  std::vector<double> sum_log_bridging_weight(set_a->sample_count);
  std::vector<double> pre_log_bridging_weight(set_a->sample_count);
  std::vector<double> set_a_log_density_prob(set_a->sample_count);
  std::vector<double> set_b_log_density_prob(set_b->sample_count);
  
  //update measurement model for set_a
  //note that set_a is resampled particle set with equal weights
  pf->current_set = set_a_idx;
  ldata.sensor->UpdateSensor(pf, (amcl::AMCLSensorData*)&ldata);
  for(int i = 0 ; i < set_a->sample_count ; ++i)
  {
    sample_a = set_a->samples + i;
    //update density probability of pi for set_a
    if(kdt == NULL)
    {
      set_a_log_density_prob[i] = sample_a->logWeight;
    }
    else
    {
      MixmclNode::poseToSe3(sample_a->pose, se3_pose);
      set_a_log_density_prob[i] = sample_a->logWeight + std::log(kdt->evaluationAt(se3_pose));
    }
    //TODO
    //update log_bridging_weight for set_a
    sum_log_bridging_weight[i] = sample_a->logWeight;//TODO average the summation  / (1.0 + iter_no)
  }

  for(int m = 1; m <= iter_no; ++m)
  {
    //apply MCMC moves and store Markov chains in set_b
    demc::proposal(set_a, demc_params, mapx, mapy, mapx_range, mapy_range, rng, set_b);
    //update measurement model for set_b
    pf->current_set = set_b_idx;
    ldata.sensor->UpdateSensor(pf, (amcl::AMCLSensorData*)&ldata);
    for(int i = 0; i < set_b->sample_count; ++i)
    {
      sample_a = set_a->samples + i;
      sample_b = set_b->samples + i;
      //update density probability of pi for set_b
      if(kdt == NULL)
      {
        set_b_log_density_prob[i] = sample_b->logWeight;
      }
      else
      {
        MixmclNode::poseToSe3(sample_b->pose, se3_pose);
        set_b_log_density_prob[i] = sample_b->logWeight + std::log(kdt->evaluationAt(se3_pose));
      }
      //update log_bridging_weight for sample_b
      //set_b_log_bridging_weight[i] = sample_b->logWeight;//TODO average the summation / (1.0 + iter_no) 
      //update acceptance probability of sample_a and sample_b
      //sample_b is numerator sample_a is denominator
      log_alpha = set_b_log_density_prob[i] - set_a_log_density_prob[i]
      if(log_alpha < 0)
        log_uniform = std::log(rng.uniform01());
      else
        log_uniform = 0;

      //if accept sample_b
      if(log_uniform <= log_alpha)
      {
        //copy sample_b to sample_a
        sample_a->pose = sample_b->pose;
        sample_a->weight = sample_b->weight;
        sample_a->logWeight = sample_b->logWeight;
        set_a_log_density_prob[i] = set_b_log_density_prob[i];
      }
      else 
      {
        //update sample_a->weight
        ;
      }
    }
  }
  //TODO normalization and accumulate weight
  
  return total;
}

} // namespace
#endif //DEMC_H
