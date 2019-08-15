
#include "ParticleStats.h"

namespace airMCL{

/*Utility function used to convert a State to cv::Vec3d
 *@param The state we want to convert
 *@return The cv::Vec3d obtained from the received state
*/
cv::Vec3d ParticleStats::stateToVec(const State& st){
    cv::Vec3d vec = cv::Vec3d();

    vec[0]=st.x;
    vec[1]=st.y;
    vec[2]=st.theta;

    return vec;
}

/*Calculates the mean of the weights of a particle set
 *@param particles A particle set
 *@return The mean of the weights of the particle set
*/
double ParticleStats::weigthAverage(const std::vector<Particle>& particles){
    double sum=0;
    int n=particles.size();

    for(int i=0;i<n;i++)
        sum+=particles[i].weight();

    return sum/double(n);
}


/*Calculates the variance of the weights of a particle set
     *@param particles A particle set
     *@return The variance of the weights of the particle set
    */
double ParticleStats::weightVariance(const std::vector<Particle>& particles){

    double w;
    double sum=0;
    double avg = weigthAverage(particles);
    int n=particles.size();

    for(int i=0;i<n;i++){
        w=particles[i].weight();
        sum+=((w-avg)*(w-avg));
    }
    return sum/double(n-1.0);
}

/*Calculates the weighted Mean of the particle set. Weights are assumed
 *to be normalized.
 *@param particles A particle set
 *@return The average weighted state
*/
State ParticleStats::weightedMean(const std::vector<Particle>& particles){

    double w;
    int n=particles.size();
    State meanSt(0,0,0);

    for(int i=0;i<n;i++){
        w = particles[i].weight();
        meanSt = (meanSt+particles[i].state()*w);
    }

    return meanSt;
}

/*Calculates the weighted Covariance Matrix of the particle set
     *@param particles A particle set
     *@return a cv::Mat of type CV_64FC1 containing the weighted Covariance Matrix
     *of the particle set.
    */
cv::Mat ParticleStats::weightedCovariance(const std::vector<Particle>& particles){

    /*Reference: http://en.wikipedia.org/wiki/Sample_covariance_matrix
     *http://en.wikipedia.org/wiki/Estimation_of_covariance_matrices*/

    int rows,cols;
    rows=cols=3;
    double w;
    double weightFactor;
    cv::Mat cov(rows,cols,CV_64FC1);
    State meanSt = weightedMean(particles);

    int n=particles.size();

    /*Calculate the weighting factor, weights are normalized, they add up to one,
     *so we only need the sum of squared weights*/
    double sumSqrdW = 0;
    for(int i=0;i<n;i++)
        sumSqrdW+= (particles[i].weight())*(particles[i].weight());

    weightFactor = 1/(1-sumSqrdW);


    double sum;
    cv::Vec3d st;
    cv::Vec3d meanV = stateToVec(meanSt);
    /*Calculate all the entries in the Covariance Matrix*/
    for(int j=0;j<rows;j++){
        for(int k=0;k<cols;k++){
            sum=0;
            for(int i=0;i<n;i++){
                w=particles[i].weight();
                st= stateToVec(particles[i].state());
                sum+=w*(st[j]-meanV[j])*(st[k]-meanV[k]);
            }
            cov.at<double>(j,k)=weightFactor*sum;
        }
    }

    return cov;
}

}
