/*Code NOT in use*/


#ifndef PARTICLEFILTER_H
#define	PARTICLEFILTER_H
#include "particle.h"
#include "../RobotFeatures/RobotAirplane.h"
#include "../Models/Models.h"
#include "../LikelihoodMeasurements/Likelihood.h"
#include "../Distributions/ParticleStats.h"


/*TODO

Methods to get/set the degeneracy threshold - Done
Method to assign a particle set to the filter - Done
*/

#define COUNTER_TO_INJECT       30
#define CUMMULATIVE_THRESHOLD   0.90

namespace airMCL
{
    /**
     * Class that implements the particle filter using SIR algorithm 
     */
    class ParticleFilter
    {
        public:
            
            /**
             * Filter constructor through params. 
             * @param nParticles Number of particles to be use in the filter.
             * @param dist0 A priori distribution from which create the first particles.
             * @param tr Motion model for transition between states.
             * @param hood Likelihood meassure to be use to match observations.
             */
            ParticleFilter
            ( int nParticles, genericDistribution* dist0, 
              motionModel* tr,  airMCL::Likelihood* hood );
            
            /**
             * Default constructor.
             */
            ParticleFilter()
            : N(0), n0(1) { }
            
            /**
             * Copy constructor.
             * @param pf Other Particle filter object.
             */
            ParticleFilter(const ParticleFilter& pf)
            : particles(pf.particles), 
              prevParticles(pf.prevParticles),N(pf.N) {  }
            
            /**
             * Do prediction step with control. Here control may refer to the 
             * control applied to velocities for the motion model.
             * @param ut Control to be applied to the motion model.
             */
            void predictionStep(const cv::Mat& ut);
            
            /**
             * Do coorection step using the current observation of the robot
             * airplane and the map.
             * @param zt Current observation of the airplane (image)
             * @param map Map where the robot is localizing itself
             */
            void correctionStep(cv::Mat& zt, cv::Mat& map);
            
            /**
             * Method to apply resampling using the low variance resampling algorithm.
             */
            void resampling();
            
            /**
             * Method to normalize weights affter applying the correction step
             */
            void normalizeWeights();
            
            /**
             * Method to obtain a constant reference of the set of particles
             * @return 
             */
            const std::vector<Particle>& getParticles()
            { return particles; }

            /* Method to assign to the filter a external created set of particles
             * @param _particles An external created particle set
             */
            void setParticles(std::vector<Particle>& _particles){
              particles = std::vector<Particle>(_particles);
              N = particles.size();
            }

            /**
             * Method to get the degeneracy threshold of the filter
             * @return The degeneracy threshold of the filter
             */
            int getDegenThreshold()
            { return n0;}

            /**
             * Method to set the degeneracy threshold of the filter
             * @param _n0 The new degeneracy threshold of the filter
             */
            void setDegenThreshold(int _n0)
            { n0=_n0; }
            
            
            std::vector<Particle> getMostSignificative();
            

        private:

            /**
             * Method to measure the level of degeneracy of the particle set
             * @return true If the filter particle set is denegerate, i.e.
             * NEff < n0, false otherwise
             */
            bool isFilterDegenerate();
            
            // Set of particles
            std::vector<Particle> particles;
            
            // Another set of particles to perform resampling
            std::vector<Particle> prevParticles;
            
            // Number of particles
            int N;
            
            // Particle degeneration threshold
            int n0;	    	

            // Motion model for transition
            motionModel* tr;
            
            // Prior distribution
            genericDistribution* p0;
            
            // Likelihood for meassurement evaluation
            airMCL::Likelihood* hood;
            
            // Airplane observation
            cv::Mat planeObs;
            
            // Number of random particles to inject
            int randomInject;
            
            // Number of iterations where had been resample
            int samplerCounter;
    };
    
    bool operator<(const Particle& p1, const Particle& p2);
}



#endif	/* PARTICLEFILTER_H */

