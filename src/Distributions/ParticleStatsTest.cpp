/*Test File for the ParticleStats class*/

#define _USE_MATH_DEFINES

#include "ParticleStats.h"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

using namespace airMCL;

int main(){

    int nParticles = 100;
    std::vector<Particle> particles;
    srand(time(NULL));

    double x,y,theta,w;

    /*Create some random particles*/
    for(int i=0;i<nParticles;i++){
        x = 100.0*rand()*1.0/RAND_MAX;
        y = 100.0*rand()*1.0/RAND_MAX;
        theta = 2*M_PI*rand()/RAND_MAX;
        w = rand()*1.0/RAND_MAX;
        particles.push_back(Particle(State(x,y,theta),w));
    }

    /*Normalize Weigths*/
    double wSum=0;
    for(int i=0;i<nParticles;i++)
        wSum+=particles[i].weight();

    for(int i=0;i<nParticles;i++)
        particles[i].weight(particles[i].weight()/wSum);

    /*Test the functions*/
    ParticleStats ps;

    double wVariance = ps.weightVariance(particles);
    State wMean = ps.weightedMean(particles);
    cv::Mat wCov = ps.weightedCovariance(particles);

    std::cout<<"Weight variance: "<< wVariance <<std::endl<<std::endl;
    std::cout<<"wMeanX: "<< wMean.x <<std::endl;
    std::cout<<"wMeanY: "<< wMean.y <<std::endl;
    std::cout<<"wMeanTheta: "<< wMean.theta <<std::endl<<std::endl;

    std::cout<<"Weighted Covariance Matrix"<<std::endl;
    /*Print the Weigthed Covariance Matrix*/
    for(int i=0;i<wCov.cols;i++){
        for(int j=0;j<wCov.rows;j++){
            std::cout<<wCov.at<double>(j,i)<<" ";
        }
        std::cout<<std::endl;
    }

    /*Marginalize the Covariance Matrix. Exclude Theta*/
    cv::Range rangeX(0,2);
    cv::Range rangeY(0,2);
    cv::Mat covXY = wCov(rangeX,rangeY);

    /*Calculate eigenValues and eigenVectors or covXY*/
    cv::Mat eigenVals,eigenVecs;
    cv::eigen(covXY,eigenVals,eigenVecs);

    std::cout<<std::endl;
    std::cout<<"Eigen Values"<<std::endl;
    for(int i=0;i<eigenVals.cols;i++){
        for(int j=0;j<eigenVals.rows;j++){
            std::cout<<eigenVals.at<double>(j,i)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;

    std::cout<<"Eigen Vectors"<<std::endl;
    for(int i=0;i<eigenVecs.cols;i++){
        for(int j=0;j<eigenVecs.rows;j++){
            std::cout<<eigenVecs.at<double>(j,i)<<" ";
        }
        std::cout<<std::endl;
    }

    return 0;
}
