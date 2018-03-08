#ifndef LASER_FEATURE_H
#define LASER_FEATURE_H
#include <cstdlib>
#include <vector>
#include "ros/ros.h"
#include "amcl/sensors/amcl_laser.h"
typedef struct laser_feature laser_feature_t;

static laser_feature_t polygonCentroid(const amcl::AMCLLaserData& ldata);

struct laser_feature
{
  // position of polygon centroid relative to the laser
  double x;
  double y;
  // average distance of all laser beams
  double dist;
};

inline static double singleArea(std::vector<double> p1, std::vector<double> p2)
{ return p1[0]*p2[1] - p2[0]*p1[1];};

inline static double addMultiply(double p1, double p2, double singleA)
{ return (p1 + p2) * singleA;};

laser_feature_t polygonCentroid(const amcl::AMCLLaserData& ldata)
{
  laser_feature_t feature;
  std::vector< std::vector<double> > vertices(ldata.range_count, std::vector<double>(2));
  int i;
  double average = 0;
  ROS_DEBUG("converting ldata into vertices.");
  for(i = 0 ; i < ldata.range_count ; ++i)
  {
    vertices[i][0] = ldata.ranges[i][0]*cos(ldata.ranges[i][1]);
    vertices[i][1] = ldata.ranges[i][0]*sin(ldata.ranges[i][1]);
    average+=ldata.ranges[i][0];
  }
  average /= ldata.range_count;

  ROS_DEBUG("calculating polygon centroid");
  double area = 0, a;
  double cx = 0, cy = 0;
  for(i = 0 ; i < ldata.range_count-1 ; ++i)
  {
    a = singleArea(vertices[i], vertices[i+1]);
    cx += addMultiply(vertices[i][0], vertices[i+1][0], a);
    cy += addMultiply(vertices[i][1], vertices[i+1][1], a);
    area += a;
  }
  a = singleArea(vertices[ldata.range_count-1], vertices[0]);
  cx += addMultiply(vertices[ldata.range_count-1][0], vertices[0][0], a);
  cy += addMultiply(vertices[ldata.range_count-1][1], vertices[0][1], a);
  area += a;
  area /=2.0;
  cx /= (6.0*area);
  cy /= (6.0*area);
  vertices.clear();
  ROS_DEBUG("finished feature calculating");
  feature.dist = average;
  feature.x = cx;
  feature.y = cy;
  return feature;
}

#endif //LASER_FEATURE_H
