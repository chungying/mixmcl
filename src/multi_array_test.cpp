#include <vector>
#include "ros/ros.h"
#include "stamped_std_msgs/StampedFloat64MultiArray.h"
#include "stamped_std_msgs/StampedUInt16MultiArray.h"

typedef struct
{
  int x;
  int y;
  int a;
  double v;
}Cell;

void assignMessage(const std::vector<Cell>& grid, stamped_std_msgs::StampedUInt16MultiArray& idx_msg, stamped_std_msgs::StampedFloat64MultiArray& data_msg, int free_space_no, int ares = 3)
{
  data_msg.array.layout.dim.resize(2);
  data_msg.array.layout.dim[0].label = "positional";
  data_msg.array.layout.dim[0].size = free_space_no;
  data_msg.array.layout.dim[0].stride = free_space_no*ares;
  data_msg.array.layout.dim[1].label = "angular";
  data_msg.array.layout.dim[1].size = ares;
  data_msg.array.layout.dim[1].stride = ares;
  data_msg.array.data.resize(free_space_no * ares);
  idx_msg.array.layout.dim.resize(2);
  idx_msg.array.layout.dim[0].label = "positional";
  idx_msg.array.layout.dim[0].size = free_space_no;
  idx_msg.array.layout.dim[0].stride = free_space_no*2;
  idx_msg.array.layout.dim[1].label = "index";
  idx_msg.array.layout.dim[1].size = 2;
  idx_msg.array.layout.dim[1].stride = 2;
  idx_msg.array.data.resize(free_space_no * 2);
  for(int i = 0 ; i < free_space_no; ++i)
  {
    idx_msg.array.data[i*2] = grid[i*ares].x;
    idx_msg.array.data[i*2 + 1] = grid[i*ares].y;
    for(int a = 0 ; a < ares; ++a)
    {
      data_msg.array.data[i*ares+a] = grid[i*ares+a].v;
      //data_msg.array.data[i*ares+a] = (double)i;
    }
  }
}

int main(int argc, char** argv)
{
  //TODO make a set of data
  int ares = 3;
  std::vector<int> x_indices;
  x_indices.push_back(1);
  x_indices.push_back(3);
  std::vector<int> y_indices;
  y_indices.push_back(2);
  y_indices.push_back(4);
  std::vector<Cell> grid;
  grid.resize(x_indices.size() * y_indices.size() * ares);
  int free_space_no = 0;
  for(int i = 0; i < x_indices.size(); ++i)
  {
    for(int j = 0; j < y_indices.size(); ++j)
    {
      for(int k = 0; k < ares; ++k)
      {
        Cell& c = grid[free_space_no*ares + k];
        c.x = x_indices[i];
        c.y = y_indices[j];
        c.a = k;
        c.v = (double)(c.x+1.0) * (c.y+1.0);
      }
      ++free_space_no;
    }
  }

  ros::init(argc, argv, "multi_array_publisher");
  ros::NodeHandle nh;
  ros::Publisher data_pub = nh.advertise<stamped_std_msgs::StampedFloat64MultiArray>("griddata",1);
  ros::Publisher idx_pub = nh.advertise<stamped_std_msgs::StampedUInt16MultiArray>("index",1);
  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok())
  {
    stamped_std_msgs::StampedFloat64MultiArray data_msg;
    stamped_std_msgs::StampedUInt16MultiArray idx_msg;
    assignMessage(grid, idx_msg, data_msg, free_space_no, ares);
    ros::spinOnce();
    data_pub.publish(data_msg);
    idx_pub.publish(idx_msg);
    loop_rate.sleep();
    ++count;
  }
}
