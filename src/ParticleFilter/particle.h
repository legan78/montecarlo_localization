

#ifndef PARTICLE_H
#define	PARTICLE_H
#include "../RobotFeatures/State.h"
#include <omp.h>

namespace airMCL
{
    
    class Particle
    {
        public:
            
            /*
             * Constructors
             */
            Particle()
            : xt(State()), wt(0), c(0), lt(0)
            {  }
            
            Particle(const Particle& p)
            : xt(p.xt), wt(p.wt), c(p.c), lt(p.lt)
            {  }
            
            Particle(const State& xt, double wt, double c=0, double l=0)
            : xt(xt), wt(wt), c(c), lt(l)
            { }
             
            ~Particle()
            { }
            
            /*
             * Operator of assignation
             */
            Particle& operator=(const Particle& other)
            {
                lt = other.lt;
                xt = other.xt;
                wt = other.wt;
                c  = other.c;
                
                return *this;
            }
            
            /*
             * Get methods
             */
            const State& state()const
            { return xt; }
            
            const double& weight()const
            { return wt; }
            
            const double& cummulative()const
            { return c; }
            
            const double& likelihood()const
            { return lt; }
            
            
            /*
             * Set methods
             */
            void state(const State& st)
            { xt=st; }
            
            void weight(double w)
            {  wt=w; }
            
            void cummulative(double _c)
            { c=_c; }
            
            void likelihood(double l)
            { lt=l; }
            
        private:
            
            // Hypothetical state
            State xt;
            
            // Importance factor
            double wt;
            
            // Cummulative importance
            double c;
            
            // Likelihood
            double lt;
            
            // 
            int associatedMod;
    };
    
    
    
}

#endif	/* PARTICLE_H */