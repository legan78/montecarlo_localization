/* 
 * File:   Models.h
 * Author: arturo
 *
 * Created on 3 de abril de 2013, 09:50 AM
 */

#ifndef MODELS_H
#define	MODELS_H
#include <opencv.hpp>
#include "../Distributions/Distributions.h"
#include "../RobotFeatures/State.h"

namespace airMCL
{
  /*
   * Abstract class modeling transition functions.
   */
  class transitionFunctionControl
  {
  public:
    // Transition function, we get an state Xt-1 and a control Ut and
    // generates a new state Xt
    virtual cv::Mat g(cv::Mat X, cv::Mat U) = 0;
  };

  /**
   * We extend the transitionFunctionControl class with the derivates
   * respect to the state and the control
   */
  class transitionDerivFunctionControl : public transitionFunctionControl
  {
  public:
    // Jacobian function w.r.t. state (Derivate of "g" w.r.t. "x"
    virtual cv::Mat dgdx(cv::Mat X, cv::Mat U)  = 0;
    // Jacobian function w.r.t. control (Derivate of "g" w.r.t. "u"
    virtual cv::Mat dgdu(cv::Mat X, cv::Mat U) = 0;
  };

  /*
   * Abstract class modeling motion models
   */
  class motionModel
  {
  public:
    // Method to generate a new sample based on the motion model
    // from a previous pose (state) and a control
    virtual State generate(const State&, const cv::Mat&) = 0;
    // Method to evaluate the probability of a hypothesis based on the motion model
    virtual double evaluate(const State&, const State&, const cv::Mat&) = 0;
  };

  /*
   * Concrete class of motion model implementing Dead Reckoning
   */
  class deadReckoningMotionModel : public motionModel
  {
  public:
    // Constructor based on Noises over movement
    deadReckoningMotionModel(double sigmaNoiseV, double sigmaNoiseO, double sigmaNoiseT, cv::Size mSize, double meSize);
    // Destructor
    ~deadReckoningMotionModel();
    // Transition function
    cv::Mat g(cv::Mat X, cv::Mat U);
    // Derivative function's
    cv::Mat dgdx(cv::Mat X, cv::Mat U); // Derivative w.r.t. X
    cv::Mat dgdu(cv::Mat X, cv::Mat U); // Derivative w.r.t. U

    // IMPLEMENTATION OF MOTION MODEL

    // Method to sample the motion model
    State generate(const State&, const cv::Mat&);
    // Method to evaluate the prob. of the motion model given a pose
    double evaluate(const State&, const State&, const cv::Mat&);
  private:
    normalUnivariateDistribution *nV; // Noise on Velocity
    normalUnivariateDistribution *nO; // Noise on Orientation
    normalUnivariateDistribution *nT; 
    // Standard deviations on Noises
    double sV;
    double sO;
    double sT;
    // Covariance matrix on parameters
    cv::Mat SU;// = cv::Mat(3,3, CV_64FC1);
    // We declare this variable to help in the detection of straight lines
    double omegaold;
    // To avoid problems with the size and orientation of the region to get samples (measures)
    // over the map, we need the size of the map and the measures.
    cv::Size mapSize;
    double measureSize; // We suppose the measure is uniform in his sides
    double limit; // Given that we can variate the orientation of the robot, we need to get
                  // a limit over the diagonal of the square formed by the measure
    // To define the boundaries of the measure region
    double xmin, xmax; 
    double ymin, ymax;
  };
}

#endif
