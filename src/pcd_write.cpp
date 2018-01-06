#include <iostream>
#include <pcl/kdtree/flann.h>
#include <boost/smart_ptr.hpp>

using namespace std;
using namespace flann;
typedef ::flann::L2_Simple<unsigned int> Dist;
typedef ::flann::Index<Dist> FLANNIndex;

int test2 (int argc, char** argv);
void setupKDTree();
void nnSearch(unsigned int x, unsigned int y, unsigned int d);

int main (int argc, char** argv)
{
  return test2(argc, argv);
}

static const int dim_ = 3;
int data_count_ = 27;
boost::shared_ptr<FLANNIndex> flann_index_;
unsigned int* data_matrix_;

int test2 (int argc, char** argv)
{
  if(argc >= 1 && argc != 4)
  {
    cout << "argc is " << argc << endl;
    cout << "ex: pcd_write_test [x] [y] [d]" << endl;
    return 1;
  }
  
  unsigned int x = 0.1;
  unsigned int y = 0.1;
  unsigned int d = 0.1;
  if(argc == 4)
  {
    x = atof(argv[1]);
    y = atof(argv[2]);
    d = atof(argv[3]);
  }
  setupKDTree();
  nnSearch(x, y, d);
}

void setupKDTree()
{
  data_matrix_= new unsigned int[dim_*data_count_];
  //TODO assign data_matrix
  for(int i = 0 ; i < dim_ ; ++i)
  {
    for(int j = 0 ; j < dim_ ; ++j)
    {
      for(int k = 0 ; k < dim_ ; ++k)
      {
        //TODO assign value
        unsigned int* pose = data_matrix_ + dim_*(i*dim_*dim_ + j*dim_ + k);
        pose[0] = i;
        pose[1] = j;
        pose[2] = k;
      }
    }
  }
  unsigned int* ptr = data_matrix_;
  //for(int i = 0;  i < data_count_ ; ++i)
  //{
  //  cout << i+1 << "-th: " << *(ptr) << ", " << *(ptr+1) << ", " << *(ptr+2) << endl;
  //  ptr += dim_;
  //}
  ptr = NULL;
  flann_index_.reset(
    new FLANNIndex(
      Matrix<unsigned int>(
        data_matrix_,
        data_count_,
        dim_
      ),
      KDTreeSingleIndexParams(15)
    )
  );
  flann_index_->buildIndex();
}

vector<unsigned int> query_point_(dim_);
static const int k_ = 1;
vector<int> k_indices_(k_);
vector<float> k_distances_(k_);
SearchParams params_;
void nnSearch(unsigned int x = 0.1, unsigned int y = 0.1, unsigned int d = 0.1)
{
  query_point_[0] = x;
  query_point_[1] = y;
  query_point_[2] = d;
  Matrix<unsigned int> query_mat (&query_point_[0], 1, dim_);
  Matrix<int> k_indices_mat (&k_indices_[0], 1, k_);
  Matrix<float> k_distances_mat (&k_distances_[0], 1, k_);
  flann_index_->knnSearch(
    query_mat, 
    k_indices_mat, 
    k_distances_mat, 
    k_, params_);
  cout << "query point: ";
  for(int i = 0 ; i < query_point_.size() ; ++i)
    cout << query_point_[i] << " ";
  cout << endl << "indices: ";
  for(int i = 0 ; i < k_indices_.size() ; ++i)
    cout << k_indices_[i] << " ";
  cout << endl << "distances: ";
  for(int i = 0 ; i < k_distances_.size() ; ++i)
    cout << k_distances_[i] << " ";
  cout << endl << "nearest pose:" 
  << *(data_matrix_ + dim_*k_indices_[0] ) << " " 
  << *(data_matrix_ + dim_*k_indices_[0] + 1) << " " 
  << *(data_matrix_ + dim_*k_indices_[0] + 2) << " " 
  << endl;
}
