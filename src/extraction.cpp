#include <iostream>
#include "tf/tf.h"
#include "nuklei/Kernel.h"
#include "amcl/pf/pf.h"
#include "amcl/pf/pf_vector.h"
using namespace std;
int main(int argc, char** argv)
{
  if(argc != 4)
    return 1;
  //TODO initial a pf_vector_t vec_p
  pf_vector_t vec_p;
  vec_p.v[0] = atof(argv[1]);
  vec_p.v[1] = atof(argv[2]);
  vec_p.v[2] = atof(argv[3]);
  //TODO convert vec_p to tf::Pose tf_p
  tf::Pose tf_p(
            tf::createQuaternionFromYaw(vec_p.v[2]),
            tf::Vector3(
              vec_p.v[0],
              vec_p.v[1],
              0.0));
  //TODO initial a se3 kernel se3_p with tf_p
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
  //TODO convert se3_p to vec_p2
  pf_vector_t vec_p2;
  vec_p2.v[0] = se3_p.loc_.X();
  vec_p2.v[1] = se3_p.loc_.Y();
  nuklei::Matrix3 mat;
  se3_p.ori_.ToRotationMatrix(mat);
  double ax, ay, az;
  //TODO check the order ZYX
  mat.ExtractEulerZYX(az, ay, ax);
  vec_p2.v[2] = (double)az;
  cout << "vec_p: " << vec_p.v[0] << ' ' << vec_p.v[1] << ' ' << vec_p.v[2] << endl;
  cout << "vec_p2: " << vec_p2.v[0] << ' ' << vec_p2.v[1] << ' ' << vec_p2.v[2] << endl;
  cout << "tf_p quaternion: " << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ", tf_p point: " << p.x() << ' ' << p.y() << ' ' << p.z() << endl;
  cout << "se3_p: " << se3_p << endl;
  return 0;
}

