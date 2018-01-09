#include <iotest.h>

#include <io/paramio.h>
#include <io/dataio.h>

#include "mixmcl/KCGrid.h"
#include "tf/tf.h"
#include "nuklei/Kernel.h"
#include "amcl/pf/pf.h"
#include "amcl/pf/pf_vector.h"
using namespace std;

//basic fstream write and read.
void TEST1();

//paramio::ParaIn read parameters from filename into a std::map<string, boost::any>
int TEST2(int argc, char** argv);

//dataio::DataIn read data from filename
void TEST3(string& filename);

//dataio::DataOut write data to filename
void TEST4(string& filename, const int n);

//test boost::ptr_map
void TEST5();

//test conversion betwen pf_vector_t, kernel::se3, tf::Pose and tf::transform
int TEST6(int argc, char** argv);

int main(int argc, char** argv)
{
  return TEST6(argc, argv);
  //TEST5();

  //string f(argv[1]);
  //return TEST2(argc, argv);

//  TEST4(f,10);
//  TEST3(f);

}

void TEST1()
{
  string filename = "test.bin";
  int hei = 3, wid = 2;
  writeExample(filename, hei, wid);

  cout << "reading test.txt" << endl;
  readExample(filename, hei, wid);
}

int TEST2(int argc, char** argv)
{
  if(argc >= 1 && argc != 2 && argc != 5)
  {
    cout << "argc is " << argc << endl;
    cout << "iotest filename [x resolution] [y resolution] [d resolution]" << endl;
    return 1;
  }
  string filename(argv[1]);
  
  size_t fxres = 10;
  size_t fyres = 10;
  size_t fdres = 10;
  if(argc == 5)
  {
    fxres = atoi(argv[2]);
    fyres = atoi(argv[3]);
    fdres = atoi(argv[4]);
  }
  cout << "TEST2" << endl;
  cout << "create ParamIn for " << filename << endl;
  boost::scoped_ptr<paramio::ParamIn> read(
    new paramio::ParamIn(filename));

  if(!read->readAllLines())
  {
    cout << "reading failed" << endl;
    return 1;
  }
  cout << "ParamIn.map_.size() is " << read->map_.size() << endl;
      for(auto i = read->map_.begin() ; i!=read->map_.end() ;++i)
      {
        if(paramio::isString(i->second))std::cout << i->first << " " << boost::any_cast<std::string>(i->second) << std::endl; 
        if(paramio::isDouble(i->second))std::cout << i->first << " " << boost::any_cast<double>(i->second) << std::endl; 
      }
  map<string, boost::any> m = read->map_;
  cout << "m.size() is " << m.size() << endl;
  //example to initialize a KCGrid object 
  //which has a member variable of KernelCollection ptr_map.
  boost::shared_ptr<KCGrid> g2(
    new KCGrid(fxres, fyres, fdres, m));
  cout << "KCGrid g2 has " << g2->data_count() << " data and " << g2->trees_count() << " trees" << endl;
  //another example to create KCGrid
  //KCGrid g3(fxres, fyres, fdres, *read);
  //cout << "KCGrid g3 has " << g3.size() << " cells" << endl;
}

void TEST3(string& filename)
{
  cout << "TEST3" << endl;
  cout << "create a dataio::DataIn object" << endl;
  cout << "filename is " << filename << endl;
  dataio::DataIn read(filename);
  pf_vector_t pose;
  laser_feature_t feature;
  while(read.readALine(pose, feature))
  {
    //printout
    cout << pose.v[0] << ' ';
    cout << pose.v[1] << ' '; 
    cout << pose.v[2] << ' ';
    cout << feature.x << ' ';
    cout << feature.y << ' ';
    cout << feature.dist << endl;
  }
  
}

void TEST4(string& filename, const int n)
{
  cout << "TEST4" << endl;
  cout << "create a dataio::DataOut object" << endl;
  cout << "filename is " << filename << endl;
  dataio::DataOut write(filename);
  pf_vector_t pose;
  pose.v[0] = 0.1;
  pose.v[1] = 0.1;
  pose.v[2] = 0.1;
  laser_feature_t feature;
  feature.x = 0.2;
  feature.y = 0.2;
  feature.dist = 0.2;
  int count = 0;
  while(write.writeALine(pose, feature))
  {
    //cout << "failed writing a line" << endl;
    ++count;
    if(count >= n)
      break;
    pose.v[0] = 2*pose.v[0] ;
    pose.v[1] = 2*pose.v[1] ;
    pose.v[2] = 2*pose.v[2] ;
    feature.x = 2*feature.x;
    feature.y = 2*feature.y;
    feature.dist = 2*feature.dist;
  }
  cout << "end writing" << endl;
}

void TEST5()
{
  typedef std::map<size_t, boost::shared_ptr<nuklei::KernelCollection> > map_t;
  map_t map;
  size_t k_size = 5;
  size_t kc_size = 2;
  size_t idx = 0;
  for( ; idx < k_size ; ++idx)
  {
    size_t i = idx % kc_size;
    nuklei::kernel::se3 k;
    k.loc_.X() = idx;
    k.setWeight(idx+1);
    try
    {
      map_t::mapped_type ptr = map.at(i);
      ptr->add(k);
    }
    catch(const std::out_of_range& e)
    {
      map_t::mapped_type ptr(new nuklei::KernelCollection);
      ptr->add(k);
      map_t::value_type pair(size_t(i), ptr);
      map.insert(pair);
    }
  }
  cout << " the map size is " << map.size() << endl;
  map_t::iterator it = map.begin();
  map_t::iterator end = map.end();
  for(;it!=end;++it)
  {
    boost::shared_ptr<nuklei::KernelCollection> kkc(it->second);
    kkc->setKernelLocH(10);
    kkc->setKernelOriH(.4);
    kkc->computeKernelStatistics();
    kkc->normalizeWeights();
    //cout << "key " << it->first << "-th tree has " << kkc->size() << " kernel(s) and total weight is " << kkc->totalWeight() << endl;
    //cout << "check again totalWeight: " << std::boolalpha << (!kkc->totalWeight()) << endl;
    //for(int i = 0 ; i < kkc->size() ; ++i)
    //{
    //  nuklei::KernelCollection::Container::const_reference se3k = kkc->at(i);
    //  cout << "  " << i+1 << "-th kernel weight is " << se3k.getWeight() << endl;
    //}

    nuklei::KernelCollection samples = kkc->sample(10);
    for(int index = 0 ; index < 10 ; ++index)
    {
      cout << index+1 << "-th sample: " <<  samples.at(index) << endl;
    }
  }
}

//test conversion betwen pf_vector_t, kernel::se3, tf::Pose and tf::transform
int TEST6(int argc, char** argv)
{
  if(argc != 4)
    return 1;
  //initial a pf_vector_t vec_p
  pf_vector_t vec_p;
  vec_p.v[0] = atof(argv[1]);
  vec_p.v[1] = atof(argv[2]);
  vec_p.v[2] = atof(argv[3]);
  //convert vec_p to tf::Pose tf_p
  tf::Pose tf_p(
            tf::createQuaternionFromYaw(vec_p.v[2]),
            tf::Vector3(
              vec_p.v[0],
              vec_p.v[1],
              0.0));
  //initial a se3 kernel se3_p with tf_p
  nuklei::kernel::se3 se3_p;
  tf::Quaternion q = tf_p.getRotation();
  tf::Point p = tf_p.getOrigin();
  se3_p.loc_.X() = p.x();
  se3_p.loc_.Y() = p.y();
  se3_p.loc_.Z() = p.z();
  se3_p.ori_.W() = q.w();
  se3_p.ori_.X() = q.x();
  se3_p.ori_.Y() = q.y();
  se3_p.ori_.Z() = q.z();
  //convert se3_p to vec_p2
  pf_vector_t vec_p2;
  vec_p2.v[0] = se3_p.loc_.X();
  vec_p2.v[1] = se3_p.loc_.Y();
  nuklei::Matrix3 mat;
  se3_p.ori_.ToRotationMatrix(mat);
  double ax, ay, az;
  //check the order ZYX
  mat.ExtractEulerZYX(az, ay, ax);
  vec_p2.v[2] = (double)az;
  cout << "vec_p: " << vec_p.v[0] << ' ' << vec_p.v[1] << ' ' << vec_p.v[2] << endl;
  cout << "vec_p2: " << vec_p2.v[0] << ' ' << vec_p2.v[1] << ' ' << vec_p2.v[2] << endl;
  cout << "tf_p quaternion: " << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ", tf_p point: " << p.x() << ' ' << p.y() << ' ' << p.z() << endl;
  cout << "se3_p: " << se3_p << endl;
  return 0;
}

