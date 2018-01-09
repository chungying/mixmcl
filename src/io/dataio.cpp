#include "io/dataio.h"
using namespace std;
namespace dataio
{
/**
  DataIn
**/
  DataIn::DataIn(const string& filename):
    ifs_ptr_(new ifstream(filename.c_str(), ios::in | ios::binary)),
    dataCount_(0)
  {
    //TODO check if ifs_ptr_ is ok?
    ifs_ptr_->seekg(0, ios_base::beg);
  }

  DataIn::~DataIn()
  {
    if(ifs_ptr_->is_open())
      ifs_ptr_->close();
  }

  void DataIn::test(ostream& out)
  {
    ifs_ptr_->seekg(0, ios_base::end);
    int length = ifs_ptr_->tellg();
    ifs_ptr_->seekg(0, ios_base::beg);
    double d;
    int count = 0;
    while(count <length)
    {
      (*ifs_ptr_) >> d;
      //out << d << ' ';
      ++count;
      //if(count % 6 == 5)
      //  out << endl;
    }
    out << "file size: " << length << ", read: " << count << endl;
  }

  bool DataIn::readALine(pf_vector_t& pose, laser_feature_t& feature)
  {
    size_t s = 6;
    double line[s];
    if(!ifs_ptr_->read( (char *)line, s * sizeof(double)))
    {
      //cout << "DataIn::readALine cannot read anymore." << endl;
      //cout << "DataIn::datacount_ is " << dataCount_ << endl;
      //cout << "DataIn::ifs_ptr_->gcount() is " << ifs_ptr_->gcount() << " data." << endl;
      return false;
    }
    pose.v[0] = line[0];
    pose.v[1] = line[1];
    pose.v[2] = line[2];
    feature.x = line[3];
    feature.y = line[4];
    feature.dist = line[5];
    ++dataCount_;
    return true;
  }
/**
  DataOut
**/
  DataOut::DataOut(const string& filename):
    ofs_ptr_(new ofstream(filename.c_str(), ios::out | ios::binary))
  {
    //TODO check if ofs_ptr_ is ok?
  }

  DataOut::~DataOut()
  {
    if(ofs_ptr_->is_open())
      ofs_ptr_->close();
  }
  
  bool DataOut::writeALine(const pf_vector_t& pose, const laser_feature_t& feature)
  {
    if(!ofs_ptr_->is_open()) return false;
    ofs_ptr_->write((char *)(&pose.v[0]), sizeof(pose.v[0]));
    ofs_ptr_->write((char *)(&pose.v[1]), sizeof(pose.v[1]));
    ofs_ptr_->write((char *)(&pose.v[2]), sizeof(pose.v[2]));
    ofs_ptr_->write((char *)(&feature.x), sizeof(feature.x));
    ofs_ptr_->write((char *)(&feature.y), sizeof(feature.y));
    ofs_ptr_->write((char *)(&feature.dist), sizeof(feature.dist));
    return true;
  }
}//namespace dataio
