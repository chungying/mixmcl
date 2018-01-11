#include "amcl/pf/pf.h"
#include "amcl/pf/pf_pdf.h"
#include "amcl/pf/pf_kdtree.h"
#include "amcl/pf/pf_resample.h"
#include "mcl/MCL.h"//for random number generator MCL::rng_

extern void pf_kdtree_clear(pf_kdtree_t *self);
extern void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value);
void pf_update_resample_low_variance(pf_t* pf)
{
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;
  double r,c,U;
  int m, i;
  double count_inv, total;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;
  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);
  count_inv = 1.0/set_a->sample_count;
  total = 0;
  r = MCL<void>::rng_.uniform01() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  set_b->sample_count = 0;
  m = 0;
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;
    U = r + m * count_inv;
    while(U>c)
    {
      i++;
      if(i >= set_a->sample_count)
      {
        c = set_a->samples[0].weight;
        i = 0;
        m = 0;
        U = r + m * count_inv;
        continue;
      }
      c += set_a->samples[i].weight;
    }
    m++;
    sample_b->pose = set_a->samples[i].pose;
    sample_b->weight = 1.0;
    total += sample_b->weight;
    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);
  }
  set_a->sample_count = 0;
  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2; 

  pf_update_converged(pf);
}

void pf_update_resample_pure_KLD(pf_t* pf)
{
  int i;
  double total;
  pf_sample_set_t *set_a, *set_b;
  pf_sample_t *sample_a, *sample_b;

  //double r,c,U;
  //int m;
  //double count_inv;
  double* c;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  // Build up cumulative probability table for resampling.
  // TODO: Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double*)malloc(sizeof(double)*(set_a->sample_count+1));
  c[0] = 0.0;
  for(i=0;i<set_a->sample_count;i++)
    c[i+1] = c[i]+set_a->samples[i].weight;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);
  
  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;
  while(set_b->sample_count < pf->max_samples)
  {
    sample_b = set_b->samples + set_b->sample_count++;

    // Naive discrete event sampler
    double r = MCL<void>::rng_.uniform01();
    //r = drand48();
    for(i=0;i<set_a->sample_count;i++)
    {
      if((c[i] <= r) && (r < c[i+1]))
        break;
    }
    assert(i<set_a->sample_count);

    sample_a = set_a->samples + i;

    assert(sample_a->weight > 0);

    // Add sample to list
    sample_b->pose = sample_a->pose;
    
    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count))
      break;
  }
  set_a->sample_count = 0;
 
  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++)
  {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }
  
  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2; 

  pf_update_converged(pf);

  free(c);
  return;
}


