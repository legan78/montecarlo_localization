/*Code NOT in use*/

#include "ParticleFilter.h"

namespace airMCL
{
   /**
     * Filter constructor through params. 
     * @param nParticles Number of particles to be use in the filter.
     * @param dist0 A priori distribution from which create the first particles.
     * @param tr Motion model for transition between states.
     * @param hood Likelihood meassure to be use to match observations.
     */
    ParticleFilter::ParticleFilter
    ( int nParticles, genericDistribution* dist0, 
      motionModel* _tr, airMCL::Likelihood* _hood )
    {
        // Uniform weight for each particle
        double w0 = 1.0/double(N);
        N              = nParticles;
        // Atributes of the class
        randomInject   = int(double(N)*0.02);
        samplerCounter = 0;
        hood           = _hood;
        tr             = _tr;
        n0             = 400;
        p0             = dist0;
        std::cout<<randomInject;
        
        // Initialization of the particles in the set. Each one has the same
        // weight and is porpotional to the uniform on (1,N)
        for(register int i=0; i<N; i++)
        {
            // Construction of the state using the a priori distribution
            cv::Mat X0 = dist0->generate();
            State   St = State( X0.at<double>(0,0), X0.at<double>(1,0), X0.at<double>(2,0) );

            particles.push_back(  Particle(St, w0)  );
        }
        // Copying the set to the other set
        prevParticles=particles;
    }
    
    
    /**
     * Do prediction step with control. Here control may refer to the 
     * control applied to velocities for the motion model.
     * @param ut Control to be applied to the motion model.
     */
    void ParticleFilter::predictionStep(const cv::Mat& ut)
    {
        for(unsigned int i=0; i<particles.size(); i++)
        {
            // Set the state generated from the state transition model using the 
            // previous state of the i particle and the applied control ut
            particles[i].state( tr->generate(particles[i].state(), ut) );
            // Copy each particle to the auxiliar particle set
            prevParticles[i]=particles[i];
        }
    }
    
    /**
     * Do correction step using the current observation of the robot
     * airplane and the map.
     * @param zt Current observation of the airplane (image)
     * @param map Map where the robot is localizing itself
     */
    void ParticleFilter::correctionStep(cv::Mat& planeObs, cv::Mat& map)
    {
        for(unsigned int i=0; i< particles.size(); i++)
        {
            // Create a robot for each particle and generate a meassure obtained
            // from its position
            airMCL::Airplane pcl = airMCL::Airplane( particles[i].state()  );
            cv::Mat zt = pcl.getObservation(map);
            // Obtaining the matching between the observed by the robot and the 
            // meassure obtained by the hypothetical state
            prevParticles[i].weight( hood->imgMatch(planeObs, zt) );
        }
    
        // Normalizing weights
        normalizeWeights();

	// Resample only if particle filter set is degenerate
        airMCL::ParticleStats ps;
        std::cout<<ps.weightVariance(prevParticles)<<std::endl;
	if( ps.weightVariance(prevParticles)>1e-9 ) resampling();
        else particles=prevParticles;
    }
    
    
    /**
     * Method to apply resampling using the low variance resampling algorithm.
     * The implementation here is the one proposed in the Thrun's book
     */
    void ParticleFilter::resampling()
    {
        particles.clear();
        int randSamples=0;
        
        double w0 = 1.0/double(N);
        double r = airMCL::genericDistribution::rand(0, 1.0/double(N));

        double c = prevParticles[0].weight();
        unsigned int i=0;
        unsigned int j;
        
        if(samplerCounter>COUNTER_TO_INJECT)
        { randSamples = randomInject; samplerCounter=0; }
        else samplerCounter++;
        
        
        // Iterating over the particles to obtain the new sample through 
        // a treshold of probability
        for(j=1; j<=prevParticles.size()-randSamples;j++)
        {
            double U = r+(j-1)/double(N);
            while(U>c)
            {
                i++;
                c+=prevParticles[i].weight();
            }
            // Selection of the best particle and setting its weight to the uniform
            if(i<prevParticles.size())
            particles.push_back( Particle( prevParticles[i].state(), w0 ) );
        }
        
        // injecting randSamples random particles
        for(;j<=prevParticles.size();j++)
        {
            cv::Mat X0 = p0->generate();
            State   St = State( X0.at<double>(0,0), X0.at<double>(1,0), X0.at<double>(2,0) );
            particles.push_back( Particle( St, w0 ) );
        }
    }
    
    /**
     * Method to normalize weights affter applying the correction step
     */
    void ParticleFilter::normalizeWeights( )
    {
        double sum = 0.0;
        // Sum of weights
        for(register int i=0; i<N; i++)
            sum+= prevParticles[i].weight();
        // Normalizing respect to the weight sum
        for(register int i=0; i<N; i++)
            prevParticles[i].weight( prevParticles[i].weight()/sum );
    }

    std::vector<Particle> ParticleFilter::getMostSignificative()
    {
        int i=0;
        double cummulative=0.0;
        sort( prevParticles.begin(), prevParticles.end() );
        
        while(cummulative<CUMMULATIVE_THRESHOLD)
        {    
            cummulative+=prevParticles[i++].weight();
        }
        
        return std::vector<Particle>( prevParticles.begin(), prevParticles.begin()+i );
    }
    
    
   /**
    * Method to measure the level of degeneracy of the particle set
    * @return true If the filter particle set is denegerate, i.e.
    * NEff < n0, false otherwise
    */
    bool ParticleFilter::isFilterDegenerate(){ 
        double sumSq=0;

        for(int i=0; i<N; i++)
            sumSq+= (prevParticles[i].weight())*(prevParticles[i].weight());

        /*Neff=1/sumSq*/
        return (1.0/sumSq)<n0;
    }
    
    
    bool operator<(const Particle& p1, const Particle& p2)
    { return !( p1.weight() < p2.weight() ); }
    
}
