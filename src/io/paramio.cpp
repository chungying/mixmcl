#include <algorithm>
#include <cctype>
#include "io/paramio.h"
using namespace std;
//using namespace paramio;
namespace paramio
{

/**
  ParamIn
**/
  ParamIn::ParamIn(const string& filename):
    ifs_ptr_(new ifstream(filename.c_str(), ios::in)),
    map_(),
    dataCount_()
  {
    //TODO check if ifs_ptr_ is ok?
  }
  
  ParamIn::~ParamIn()
  {
    if(ifs_ptr_->is_open())
      ifs_ptr_->close();
  }
  
  bool ParamIn::readAllLines()
  {
    if(!ifs_ptr_->is_open()) 
    {
      return false;
    }
    char ckey[64], ctype[8], cval[256];
    string key, type;
    string line;
    float dval;
    boost::any av;
    ifs_ptr_->seekg(0, ios::beg);
    dataCount_ = 0;
    while(getline(*ifs_ptr_, line))
    {
      ++dataCount_;
      sscanf(line.c_str(), "%s %s %s", ckey, ctype, cval);
      type = ctype;
      if(type == typeid(string).name())
      {
        key = ckey;
        av = string(cval);
      }
      else if(type == typeid(double).name() || type == typeid(float).name())
      {
        sscanf(cval, "%f", &dval);
        key = ckey;
        av = dval;
      }
      map_.insert(make_pair(key, av));
    }
    return true;
  }

/**
  ParamOut
**/
  ParamOut::ParamOut(const string& filename):
    ofs_ptr_(new ofstream(filename.c_str(), ios::out))
  {
    //TODO check if ofs_ptr_ is ok?
  }

  ParamOut::~ParamOut()
  {
    if(ofs_ptr_->is_open())
      ofs_ptr_->close();
  }

  bool ParamOut::writeALine(const string& key, const string& value)
  {
    if(!ofs_ptr_->is_open()) return false;
    *ofs_ptr_ << key << " " << typeid(string).name() << " " << value << endl;
    return true;
  }
  
  bool ParamOut::writeALine(const string& key, const double& value)
  {
    if(!ofs_ptr_->is_open()) return false;
    *ofs_ptr_ << key << " " << typeid(double).name() << " " << value << endl;
    return true;
  }
}//end namespace paramio

