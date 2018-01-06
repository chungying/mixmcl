#include <iostream>
#include <boost/optional/optional.hpp>
using namespace std;

int main(int argc, char** argv)
{
  boost::optional<double> w;
  w = boost::none;
  cout << "w = boost::none " << *w << " " << boolalpha << !w << endl;
  w = 0;
  cout << "w = 0 " << *w << " " << boolalpha << !w << endl;
  w = 1;
  cout << "w = 1 " << *w << " " << boolalpha << !w << endl;
  *w = 1;
  cout << "*w = 1 " << *w << " " << boolalpha << !w << endl;
}


