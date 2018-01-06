#ifndef KCGRID_H
#define KCGRID_H
#include <map>
#include "nuklei/KernelCollection.h"
#include "io/dataio.h"
#include "io/paramio.h"
#include "amcl/pf/pf_vector.h"
#include "mixmcl/laser_feature.h"
#include "boost/smart_ptr.hpp"
#include "boost/ptr_container/ptr_map.hpp"
#include "pcl/kdtree/flann.h"
#include "tf/tf.h"
//read in param.txt 
//sotre the parameters as a std::map
//initialize a 3-D discrete grid with size of (X, Y, D)
//X is the feature of X-axis value of the polygon centroid of scan points.
//Y is the feature of Y-axis value of the polygon centroid of scan points.
//D is the average distance of laser scans.
class AbstractKCGridConvertor
{
  public:
    virtual void convertKCGrid() = 0;
};

class KCGrid
{
  public:
    typedef boost::shared_ptr<nuklei::KernelCollection> TreePtr;
    typedef boost::shared_ptr<const nuklei::KernelCollection> ConstTreePtr;
    typedef std::map<size_t, TreePtr> TreeMap;
    typedef std::map<size_t, ConstTreePtr> ConstTreeMap;
    typedef ::flann::L2_Simple<float> Dist;
    typedef ::flann::Index<Dist> FLANNIndex;

    KCGrid(size_t X, size_t Y, size_t D, std::string& para_file);

    KCGrid(size_t X, size_t Y, size_t D, std::map<std::string, boost::any>& m);

    KCGrid(size_t X, size_t Y, size_t D, paramio::ParamIn& param);

    inline size_t GI2VI(size_t x, size_t y, size_t d);

    inline size_t C2VI(float x, float y, float d);

    inline void VI2GI(size_t index, std::vector<size_t>& gi);

    static inline size_t cont2disc(float cont, const std::pair<float, float>& lim, size_t res);

    static inline float disc2cont(size_t grid_index, const std::pair<float, float>& lim, size_t res);

    size_t trees_count(){return tree_map_.size();};
    size_t data_count(){return data_count_;};

    //get a tree corresponding to the feature
    TreeMap::mapped_type getTree(float x, float y, float d)
    {
      //return tree_map_.at(C2VI(x, y, d));
      return tree_map_.at(nnSearch(x, y, d));
    };

  private:
    size_t X, Y, D;
    size_t max_size_, data_count_;
    std::pair<float,float> xlim, ylim, dlim;
    std::vector<TreeMap::key_type> gridcell_indices_;
    boost::shared_ptr<float> data_matrix_;
    boost::shared_ptr<FLANNIndex> flann_index_;
    TreeMap tree_map_;

    void assignLimits(float xmin, float xmax, float ymin, float ymax, float dmin, float dmax);

    bool assignLimits( const std::map<std::string, boost::any>& m);

    void convert(std::map<std::string, boost::any>& m);

    size_t nnSearch(float x, float y, float d);
};
#endif//KCGRID_H
