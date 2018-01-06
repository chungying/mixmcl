#ifndef DATAIO_H
#define DATAIO_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <boost/scoped_ptr.hpp>

//DataType
#include "amcl/pf/pf_vector.h"//for pf_vector_t
#include "mixmcl/laser_feature.h"//for laser_feature_t

using namespace std;

namespace dataio
{
/**
  DataIn
**/
  class DataIn
  {
    public:
      DataIn(const string& filename);
      ~DataIn();
      bool readALine(pf_vector_t& pose, laser_feature_t& feature);
      void test();
    protected:
      int dataCount_;
      boost::scoped_ptr<ifstream> ifs_ptr_;
  };//end class DataIn

/**
  DataOut
**/
  class DataOut
  {
    public:
      DataOut(const string& filename);
      ~DataOut();
      bool writeALine(const pf_vector_t& pose, const laser_feature_t& feature);//pair-wise
    protected:
      boost::scoped_ptr<ofstream> ofs_ptr_;//noncopyable and gauranteed to be deleted on either destruction or reset.
  };//end class DataOut



}//namespace dataio

#endif //DATAIO_H
