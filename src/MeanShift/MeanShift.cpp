#include "MeanShift.h"


namespace airMCL
{
    
  double MeanShift::bandWidth=0.1;  
  double MeanShift::distance =30 ;  //  Pixels
  int    MeanShift::N = 50;
    
  std::vector<Particle> MeanShift::calculateMeans
  ( const std::vector<Particle> &particles, double h )
  {
    double threshold = 0.000001;

    // We create the vector of Particles copying the original vector
    // to calculate the iterative mean shift
    std::vector<Particle> means = particles;
    
    // Using a convergence criteria, we iterate the mean shift algorithm to
    // get the modes of the Density
    bool criteria = false;

    while (!criteria)
      {
	criteria = true;
	// For a given point, we evaluate the mean shift (the gradient)
	for (register int i = 0; i < means.size(); i++)
	  {
	    double s1 = 0;
	    State s2(0,0,0);
	    // We use a Gaussian kernel to calculate the Mean shift
	    for (register int j = 0; j < particles.size(); j++)
	      {
		// We evaluate the norm of the State differences
		State st = means[i].state() - particles[j].state();
		double norm = st.x*st.x + st.y*st.y + st.theta*st.theta;
		// We now evaluate the kernel profile
		s1 += exp(-norm/(h*h));
		s2 = s2 + s2*s1;
	      }
	    State s = s2/s1;
	    // We check every mean calculated to know if we already have convergence
	    if (s > threshold) criteria = false;
	    // Move the mean
	    means[i].state(s - means[i].state());
	  }
      }

    // Now we get the different means to get the modes of the density
    std::vector<Particle> result;
    result.push_back(means[0]);
    for (register int i = 1; i < means.size(); i++)
      {
	// We need to search over the result set to verify if we encounter
	// a new mode
	for (register int j = 0; j < result.size(); j++)
	  {
	    if ((means[i].state() - result[j].state()) > threshold)
	      {
		// We get a new mode
		result.push_back(means[i]);
	      }
	  }
      }

    // Finally, we return the result of the modes
    return result;
  }
  
  
  
   std::vector<Particle> MeanShift::getModes
   ( const std::vector<Particle>&  particles)
   {
       std::vector<Particle> prevModes = particles;
       std::vector<Particle> newModes  = particles;
       double c=sqrt(2*3.1416);
       double norm;
       State st;
       
       for(int k=0; k<N; k++)
       {
           norm=0.0;
           for(unsigned int i=0; i<particles.size(); i++)
           {
               double s1 = 0.0;
               double g  = 0.0, u=0;

               for(unsigned int j=0; j<particles.size(); j++)
               {
                   u = (abs(prevModes[i].state()-particles[j].state()))/bandWidth;
                   g = -u*(1.0/c)*exp(-0.5*u*u);
                   s1 += g;
                   st = st+particles[j].state()*g;
               }

               newModes[i].state( st/s1 - prevModes[i].state() );
               double nNorm=abs(newModes[i].state()-prevModes[i].state());
               norm+=nNorm*nNorm;
           }

           prevModes = newModes;
       }
       
       return merge( newModes );
   }
   
   std::vector<Particle> MeanShift::merge
   ( const std::vector<Particle>& modesNotMerged)
   {
       std::vector<Particle> modes;
       
       modes.push_back(modesNotMerged[0]);
       
       for( unsigned int i=0; i<modes.size(); i++)
       {
           for( unsigned int j=0; j<modesNotMerged.size(); j++ )
           {
               if(abs(modesNotMerged[j].state()-modes[i].state())>distance)// Euclidian distance
               {
                   modes.push_back(modesNotMerged[i]);
               }
           }
       }
       
       return modes;
   }
   
   double abs(const State& st)
   {
        return sqrt(st.x*st.x + st.y*st.y + st.theta*st.theta);
   }
  
  
}
