#ifndef PARTICLESTATISTICS_H
#define PARTICLESTATISTICS_H

#include <vector>
#include <core/core.hpp>
#include "../ParticleFilter/particle.h"
#include "../RobotFeatures/State.h"

namespace airMCL{

/*Utility class used to calculate some statistical measures over the particle set*/
class ParticleStats{
public:

    /**
     * Calculates the variance of the weights of a particle set
     * @param particles A particle set
     * @return The variance of the weights of the particle set
    */
    double weightVariance(const std::vector<Particle>& particles);

    /**
     * Calculates the weighted Mean of the particle set. Weights are assumed
     * to be normalized.
     * @param particles A particle set
     * @return The average weighted state
    */
    State weightedMean(const std::vector<Particle>& particles);

    /**
     * Calculates the weighted Covariance Matrix of the particle set
     * @param particles A particle set
     * @return a cv::Mat of type CV_64FC1 containing the weighted Covariance Matrix
     * of the particle set.
    */
    cv::Mat weightedCovariance(const std::vector<Particle>& particles);

private:
    /*Calculates the mean of the weights of a particle set
     *@param particles A particle set
     *@return The mean of the weights of the particle set
    */
    double weigthAverage(const std::vector<Particle>& particles);

    /*Utility function used to convert a State to cv::Vec3d
     *@param The state we want to convert
     *@return The cv::Vec3d obtained from the received state
    */
    cv::Vec3d stateToVec(const State& st);

};

}
#endif // PARTICLESTATISTICS_H
