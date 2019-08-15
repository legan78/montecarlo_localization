#include <iostream>
#include "Distributions.h"

using namespace std;
using namespace airMCL;

int main(int argc, char* argv[]) 
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0,10);

  normalUnivariateDistribution n(0,10);

  for (register int i = 0; i < 10; i++)
    cout << n.generate() << " ";
  cout << endl;

  for (register int i = 0; i < 10; i++)
    cout << distribution(generator) << " ";
  cout << endl;

  return 0;
}
