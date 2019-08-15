#ifndef MEANSHIFT_H
#define MEANSHIFT_H

#include <iostream>
#include "../ParticleFilter/ParticleFilter.h"

namespace airMCL
{
  class MeanShift
  {
  public: 
    static std::vector<Particle> calculateMeans
    ( const std::vector<Particle>&, double );
    
    static std::vector<Particle> getModes
    ( const std::vector<Particle>&  );
    
    static std::vector<Particle> merge
    (const std::vector<Particle>& );
    
    static double bandWidth;
    static double distance;
    static int    N;
    
  };
  
      
    double abs(const State& st);
}

#endif
