#include <iostream>
#include "amcl/pf/pf.h"
#include "random_numbers/random_numbers.h"
using namespace std;
int main(int argc, char** argv)
{
  pf_t* pf;
  pf = pf_alloc(0, 10, 0.0, 0.0, NULL, NULL);
  pf_sample_set_t *set = pf->sets;
  //pf_kdtree_free(set->kdtree);
  //set->kdtree = pf_kdtree_alloc(3000);
  pf_vector_t pose;
  random_numbers::RandomNumberGenerator rng;
  double size = 30;
  for(int i = 0 ; i < 30; ++i)
  {
    pose.v[0] = rng.uniform01()*size;
    pose.v[1] = rng.uniform01()*size;
    pose.v[2] = rng.uniform01()*2*M_PI-M_PI;
    pf_kdtree_insert(set->kdtree, pose, 0.01);
  }
  pf_kdtree_t* tree = set->kdtree;
  pf_kdtree_node_t* root = set->kdtree->root;
  cout << "node count: " << tree->node_count << ' ' << tree->node_max_count << endl;
  cout << "root: " << root << ' ' << root->key[0] << ' ' << root->key[1] << ' ' << root->key[2] << endl;
  cout << "first: " << tree->nodes << ' ' << tree->nodes->key[0] << ' ' << tree->nodes->key[1] << ' ' << tree->nodes->key[2] << endl;
  for(int i = 0 ; i < set->kdtree->node_count; ++i)
  {
    pf_kdtree_node_t* node = tree->nodes + i;
    if(node->children[0] == NULL)
      cout << "node: " << node << ' ' << (node-tree->nodes) << " NULL NULL" <<  endl;
    else
      cout << "node: " << node << ' ' << (node-tree->nodes) << ' ' << (node->children[0] - tree->nodes) << ' '<<  (node->children[1] - tree->nodes) << endl;
  }

  pf_free( pf );
}
