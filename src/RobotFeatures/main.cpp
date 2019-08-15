/* 
 * File:   main.cpp
 * Author: angel
 *
 * Created on 1 de abril de 2013, 08:24 PM
 */

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include "RobotAirplane.h"
#include "../Drawer/Drawer.h"
#include "../ParticleFilter/ParticleFilter.h"
#include "../LikelihoodMeasurements/HistDiff.h"
#include "../MeanShift/MeanShift.h"

// #define MAKE_VIDEO
using namespace std;

void getImgSamples(airMCL::ParticleFilter& filter, cv::Mat& map);
void watchVideo();
/*
 * 
 */
// TODO: Makefile to compile on console

/**
 * Constants to transfer to constants.h
 */
double pi=3.1416;
double c= (51.0)*(1.0/sqrt(2.0))+10;

int main(int argc, char** argv) {

    // Reading the map image 
    std::string file("../IKONOS_venice.jpg");    
    cv::Mat map=cv::imread(file.c_str());
    
    // Changing mao size to make it visible on screen
    cv::Mat map2, visualize;
    
    // The first state of the robot
    airMCL::State X0( airMCL::genericDistribution::rand(c,map.cols-c), 
                      airMCL::genericDistribution::rand(c,map.rows-c), 
                      airMCL::genericDistribution::rand(0, 2*pi) );
    
    // The airplane robot
    airMCL::Airplane robot(X0);
              
    // Defining ranges where to take a valid sample of state
    airMCL::uniform::MultiRange ranges;
    ranges.push_back( std::pair<double,double>(c, map.cols-c) );
    ranges.push_back( std::pair<double,double>(c, map.rows-c) );
    ranges.push_back( std::pair<double,double>(0, 2.0*pi) );
    
    // Creating a uniform distribution of state dimension
    airMCL::genericDistribution* unif = new airMCL::uniform(3, ranges);
    
    // Dead reckoning motion model
    airMCL::motionModel *motion = new airMCL::deadReckoningMotionModel( 2, 0.01, 0.1, cv::Size(map.cols, map.rows), 51.0 );
    
    // Creating and setting up the method to evaluate the observation model
    airMCL::HistDiff *histComp = new airMCL::HistDiff();
    histComp->setDiv(32);                                                       //Configure the Histogram Difference Comparion Object
    histComp->setComparisonType(CV_COMP_CORREL);                                // Comparison methods: 
                                                                                //                      CV_COMP_INTERSECT, 
                                                                   //                      CV_COMP_CHISQR, 
                                                                                //                      CV_COMP_CORREL, 
                                                                                //                      CV_COMP_BHATTACHARYYA
    // Likelihood evaluation object
    airMCL::Likelihood *imgComp;
    imgComp=histComp;
    
    // Creating the particle filter with uniform prior, a motion model and the likelihood evaluator
    airMCL::ParticleFilter filter( 3000, unif,  motion, imgComp);     
    
    // Setting up the control of robot
    cv::Mat ut = cv::Mat::zeros(2,1,CV_64FC1);
    ut.at<double>(0,0) = 30;
    ut.at<double>(1,0) = 0.01;
    
    // Particle filter state and covariance estimator
    airMCL::ParticleStats ps;                           
    airMCL::State Xt;                                   // Current state estimation
    cv::Mat       St;                                   // Current covariance estimation
    
    cv::Range r1(0,2);
    cv::Range r2(0,2);
    
    for(int i=0; i<1000; i++)
    {        
        // Image map where to draw things
        map2=map.clone();
        // Setting the new state of the robot given the new control ut
        robot.setState( motion->generate(robot.getState(), ut) );
        // Do prediction step with the control given
        filter.predictionStep(ut);
        // Get the real observation from the airplane robot
        cv::Mat cropped = robot.getObservation(map2);
        // Do correction step using the observation given 
        filter.correctionStep(cropped, map2);        
        // Obtaining the estimation of the robot estate
        Xt = ps.weightedMean( filter.getParticles() );
        St = ps.weightedCovariance( filter.getParticles() );
        
        // Drawing the particles, the robot square observation
        airMCL::Drawer::drawParticles(filter.getParticles(), map2);             // Drawing the particles
        airMCL::Drawer::drawRobotObs(robot, map2);       // Robot square observation
        airMCL::Drawer::drawRobotState(Xt, map2);                               // Robot current state estimation
        airMCL::Drawer::drawRobotState( robot.getState(), map2, 
                                         cv::Scalar(255,0,255) );
        
        airMCL::Drawer::drawCovariance(St(r1,r2), Xt, map2);
        
        // Showing the estimated state
//        std::cout<< "Estimated state: "
//                 << Xt.x <<" "
//                 << Xt.y <<" "
//                 << Xt.theta
//                 << std::endl;
        
        cv::resize(map2,visualize,cv::Size(map.rows/4, map.cols/4));
        cv::imshow("imgOrg", visualize);
        cv::imshow("Robot eyes", cropped);
        cv::waitKey(50);
        
#ifdef MAKE_VIDEO
        char str[10];
        std::sprintf(str, "%d", i+1);
        cv::imwrite(srcVideo+str+".jpg", map2);
#endif
    }
   
   // watchVideo();
    
    return 0;
}

/**
 * Method to obtain some samples to evaluate the goodness of the likelihood evaluation
 * methods
 * @param filter Particles from some filter
 * @param map Image map from where to take the samples
 */

// /home/angel/Dropbox/Master/Clases/RobProb/Tareas/Tarea5/AirplaneMCL/src/Video/src/src
void getImgSamples(airMCL::ParticleFilter& filter, cv::Mat& map)
{
    // Name of the image sample for save in file
    std::string name("/home/angel/Dropbox/Master/Clases/RobProb/Tareas/Tarea5/AirplaneMCL/imgSamples/sample");
    // Getting the samples from the particle position
    for(unsigned int i=0; i<filter.getParticles().size(); i++)
    {
        airMCL::Airplane pcl = airMCL::Airplane( filter.getParticles()[i].state()  );
        cv::Mat gaze = pcl.getObservation(map);
        // Saving to an image file
        char str[10];
        std::sprintf(str, "%d", i+1);
        cv::imwrite(name+str+".jpg", gaze);
    }
    
}


void watchVideo()
{
    std::string srcVideo("/home/robst/Desktop/src/src");
    cv::Mat visualize;
    
    for(int i=1; i<=1000; i++)
    {
        char str[10];
        std::sprintf(str, "%d", i+1);
        cv::Mat src =cv::imread((srcVideo+str+".jpg").c_str());
        cv::resize(src,visualize,cv::Size(src.rows/3, src.cols/3));
        cv::imshow("src", visualize);
        cv::waitKey(50);
    }
    

}
