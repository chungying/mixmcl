#ifndef PARAMIO_H
#define PARAMIO_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
#include <string>
#include <boost/scoped_ptr.hpp>
#include <boost/any.hpp>
#include <typeinfo>
using namespace std;

namespace paramio
{
  static bool isString(const boost::any a);
  static bool isDouble(const boost::any a);
  static bool isFloat(const boost::any a);
  class ParamOut
  {
    public:
      ParamOut(const string& filename);
      ~ParamOut();
      bool writeALine(const string& key, const double& value);//pair-wise
      bool writeALine(const string& key, const string& value);//pair-wise
    protected:
      boost::scoped_ptr<ofstream> ofs_ptr_;//noncopyable and gauranteed to be deleted on either destruction or reset.
  };//end class ParamOut

  class ParamIn
  {
    public:
      ParamIn(const string& filename);
      ~ParamIn();
      bool readAllLines();
      map<string, boost::any> map_;
    protected:
      int dataCount_;
      boost::scoped_ptr<ifstream> ifs_ptr_;
  };//end class ParamIn
  
  bool isString(const boost::any a)
  {
    const std::type_info &ti = a.type();
    return typeid(string)==ti;
  };
  
  bool isDouble(const boost::any a)
  {
    const std::type_info &ti = a.type();
    return typeid(double)==ti;
  };

  bool isFloat(const boost::any a)
  {
    const std::type_info &ti = a.type();
    return typeid(float)==ti;
  };
}//end namespace paramio

#endif //PARAMIO_H
