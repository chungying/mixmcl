#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;
void writeExample(const std::string& file, const int& hei, const int& wid);
void readExample(const std::string& file, const int& hei, const int& wid);

void writeExample(const std::string& file, const int& hei, const int& wid)
{
  
  ofstream ofs(file.c_str(), ios::out | ios::binary);
  if(!ofs)
  {
    cout << "cannot write test.txt" << endl;
  }

  double* r;
  //int wid = 2;
  //int hei = 3;
  int size = wid*hei;
  r = new double[size];
  int idx;
  for(int i = 0 ; i < hei ; i++)
  {
    for(int j = 0 ; j < wid ; j++)
    {
      idx = i*wid + j;
      r[idx] = drand48();
      cout << r[idx] << " ";
      ofs.write((char *)(&r[idx]), sizeof(double));
      //ofs.write((char *)(r+idx), sizeof(double));
    }
    cout << endl;
  }
//  ofs.write((char *)r, sizeof(double)*size);
  delete[] r;
  ofs.close();
  
}

void readExample(const std::string& file, const int& hei, const int& wid)
{
  ifstream ifs(file.c_str(), ios::in | ios::binary | ios::ate);
  if(!ifs)
  {
    cout << "cannot read test.txt" << endl;
  }
  streampos s = ifs.tellg();
  int ss = s/sizeof(double);
  double* entry = new double[ss];
  ifs.seekg(0, ios::beg);
  ifs.read((char *)entry, s);
  ifs.close();

  int count = 0;
  for(int i = 0 ; i < hei ; i++)
  {
    for(int j = 0 ; j < wid ; j++)
    {
    cout << entry[count] << " ";
    count++;
    if(count%wid==0)cout << endl;
    }
  }
}
