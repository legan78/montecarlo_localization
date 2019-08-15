#include <cmath>
#include "Models.h"

namespace airMCL
{
  deadReckoningMotionModel::deadReckoningMotionModel(double sigmaNoiseV, double sigmaNoiseO, double sigmaNoiseT, 
						     cv::Size mSize, double meSize) : sV(sigmaNoiseV), sO(sigmaNoiseO), sT(sigmaNoiseT), omegaold(1.0f), mapSize(mSize), measureSize(meSize)
  {
    nV = new normalUnivariateDistribution(0,sigmaNoiseV*sigmaNoiseV);
    nO = new normalUnivariateDistribution(0,sigmaNoiseO*sigmaNoiseO);
    nT = new normalUnivariateDistribution(0,sigmaNoiseT*sigmaNoiseT);
    SU = cv::Mat::zeros(3,3, CV_64FC1);
    SU.at<double>(0,0) = sigmaNoiseV*sigmaNoiseV;
    SU.at<double>(1,1) = sigmaNoiseO*sigmaNoiseO;
    SU.at<double>(2,2) = sigmaNoiseT*sigmaNoiseT;
    // We calculate the half of the diagonal of the square given the measure size
    limit = measureSize/sqrt(2); // We suppose the measure it's an square
    xmin = limit;
    ymin = limit;
    xmax = mapSize.width - limit;
    ymax = mapSize.height - limit;
  }

  deadReckoningMotionModel::~deadReckoningMotionModel()
  {
    delete nV;
    delete nO;
    delete nT;
  }

  // Transition function
  cv::Mat deadReckoningMotionModel::g(cv::Mat X, cv::Mat U)
  {
    cv::Mat nX = cv::Mat(3,1, CV_64FC1);
    
    // State
    double x = X.at<double>(0,0);
    double y = X.at<double>(1,0);
    double t = X.at<double>(2,0); // Theta
    // Control
    double v = U.at<double>(0,0);
    double w = U.at<double>(1,0);
    // We calculate the new state
    if (fabs(w) > 0.000001)
      {
	double r = v/w;
	nX.at<double>(0,0) = x - r*sin(t) + r*sin(t+w); // x 
	nX.at<double>(1,0) = y + r*cos(t) - r*cos(t+w); // y 
	nX.at<double>(2,0) = t + w; // Don't consider case where Control is 3 dimensional
      }
    else // w it's almost zero
      {
	nX.at<double>(0,0) = x + v*cos(t); // x
	nX.at<double>(1,0) = y + v*sin(t); // y
	nX.at<double>(2,0) = t + w;
      }
    // Return the new state
    return nX;
  }

  // We implement the derivate w.r.t. X of the motion model
  cv::Mat deadReckoningMotionModel::dgdx(cv::Mat X, cv::Mat U)
  {
    cv::Mat J = cv::Mat::eye(3,3, CV_64FC1);

    // State
    double t = X.at<double>(2,0); // Theta
    // Control
    double v = U.at<double>(0,0);
    double w = U.at<double>(1,0);
    // We calculate the derivate
    if (fabs(w) > 0.000001)
      {
	double r = v/w;
	J.at<double>(0,2) = -r*cos(t) + r*cos(t+w);
	J.at<double>(1,2) = -r*sin(t) + r*sin(t+w);
      }
    else // w it's almost zero
      {
	J.at<double>(0,2) = -v*sin(t);
	J.at<double>(1,2) =  v*cos(t);
      }
    // return the Jacobian matrix
    return J;
  }

  // We implement the derivate w.r.t. U of the motion model
  cv::Mat deadReckoningMotionModel::dgdu(cv::Mat X, cv::Mat U)
  {
    cv::Mat J = cv::Mat(3,3, CV_64FC1);
    // State
    double t = X.at<double>(2,0);
    // Control
    double v = U.at<double>(0,0);
    double w = U.at<double>(1,0);
    // We calculate the derivate
    if (fabs(w) > 0.000001)
      {
	double r = v/w;
	double dr = -v/(w*w);
	J.at<double>(0,0) = (-sin(t) + sin(t+w))/w;
	J.at<double>(0,1) = (-sin(t) + sin(t+w))*dr + r*cos(t+w);
	J.at<double>(1,0) = ( cos(t) - cos(t+w))/w;
	J.at<double>(1,1) = ( cos(t) - cos(t+2))*dr + r*sin(t+w);
	J.at<double>(2,1) = 1.0;
	J.at<double>(2,2) = 1.0;
      }
    else // w it's almost zero
      {
	J.at<double>(0,0) = cos(t);
	J.at<double>(0,1) = 0.0;
	J.at<double>(1,0) = sin(t);
	J.at<double>(1,1) = 0.0;
	J.at<double>(2,1) = 1.0;
	J.at<double>(2,2) = 1.0;
      }
    // return the Jacobian matrix
    return J;
  }

  // We implement the sample motion model for the DeadReckoning
  State deadReckoningMotionModel::generate(const State &X, const cv::Mat &U)
  {
    // State
    double x = X.x;
    double y = X.y;
    double t = X.theta;
    // Control with noise
    double vn     = U.at<double>(0,0) + nV->generate();
    double omegan = U.at<double>(1,0) + nO->generate(); 
    // We compute the new sample
    State p;
    if (omegan*omegaold > 0) 
      {
	double r = vn/omegan;
	p.x = x - r*sin(t) + r*sin(t+omegan); // x
	p.y = y + r*cos(t) - r*cos(t+omegan); // y
	p.theta = t + omegan; // theta
      }
    else
      {
	p.x = x + vn*cos(t); // x
	p.y = y + vn*sin(t); // y
	p.theta = t; // theta
      }
    omegaold = omegan;
    // We need to verify that the new pose it's not outside the region where we cannot capture
    // measures of the image

    // We check if the angle it's in between 0 and 2*PI
    while (p.theta > 2*M_PI) p.theta -= 2*M_PI;

    // First we check if the new pose it's in the upper-left corner
    if ((p.x <= xmin) && (p.y <= ymin))
      {
	p.x = xmin + 1;
	p.y = ymin + 1;
	p.theta += M_PI;
      }
    // We check if the new pose it's in the lower-left corner
    if ((p.x <= xmin) && (p.y >= ymax))
      {
	p.x = xmin + 1;
	p.y = ymax - 1;
	p.theta -= M_PI;
      }
    // We check if the new pose it's in the lower-right corner
    if ((p.x >= xmax) && (p.y >= ymax))
      {
	p.x = xmax - 1;
	p.y = xmax - 1;
	p.theta -= M_PI;
      }
    // We check if the new pose it's in the upper-right corner
    if ((p.x >= xmax) && (p.y <= ymin))
      {
	p.x = xmax - 1;
	p.y = xmin + 1;
	p.theta += M_PI;
      }

    // Now we check if the new pose it's in a specific boundary

    // We check if the angle it's in between 0 and 2*PI
    while (p.theta > 2*M_PI) p.theta -= 2*M_PI;

    // Upper boundary
    if ((p.y <= ymin) && (p.x > xmin) && (p.x < xmax))
      {
	p.y = ymin + 1;
	if (p.theta == 3*M_PI/2) p.theta -= M_PI;
	if ((p.theta > M_PI) && (p.theta < 3*M_PI/2)) p.theta -= M_PI/2;
	if ((p.theta > 3*M_PI/2) && (p.theta < 2*M_PI)) p.theta += M_PI/2;
      }
    // Left boundary
    if ((p.x <= xmin) && (p.y > ymin) && (p.y < ymax))
      {
	p.x = xmin + 1;
	if (p.theta == M_PI) p.theta -= M_PI;
	if ((p.theta > M_PI/2) && (p.theta < M_PI)) p.theta -= M_PI/2;
	if ((p.theta > M_PI) && (p.theta < 3*M_PI/2)) p.theta += M_PI/2;
      }
    // Lower boundary
    if ((p.y >= ymax) && (p.x > xmin) && (p.x < xmax))
      {
	p.y = ymax - 1;
	if (p.theta == M_PI/2) p.theta += M_PI;
	if ((p.theta > 0) && (p.theta < M_PI/2)) p.theta += 3*M_PI/2;
	if ((p.theta > M_PI/2) && (p.theta < M_PI)) p.theta += M_PI/2;
      }
    // Right boundary
    if ((p.x >= xmax) && (p.y > ymin) && (p.y < ymax))
      {
	p.x = xmax - 1;
	if (p.theta == 0) p.theta += M_PI;
	if ((p.theta > 0) && (p.theta < M_PI/2)) p.theta += M_PI/2;
	if ((p.theta > 3*M_PI/2) && (p.theta < 2*M_PI)) p.theta -= M_PI/2;
      }
	
    return p;
  }

  // Method to get the p(xt | ut, xt-1) evaluating the PDF over the algorithm in Table 5.1
  // in Thrun's book
  double deadReckoningMotionModel::evaluate(const State &XX, const State &X, const cv::Mat &U)
  {
    // Hypothesis
    double xp = XX.x;
    double yp = XX.y;
    double tp = XX.theta;
    // State
    double x = X.x;
    double y = X.y;
    double t = X.theta;
    // Control
    double v = U.at<double>(0,0);
    double w = U.at<double>(1,0);
    // Evaluate the density
    // double p = 1.0f;
    double vv = 0.0f;
    double ww = 0.0f;
    // We evaluate the probability of being reached from p
    double den = (y-yp)*cos(t) - (x-xp)*sin(t);
    if (fabs(den) > 0)
      {
	double mu = 0.5*((xp-xp)*cos(t) + (y-yp)*sin(t))/den;
	double xc = (x+xp)/2.0 + mu*(y-yp); // x*
	double yc = (y+yp)/2.0 + mu*(xp-x); // y*
	double sc = -(xp-x)*sin(t) + (yp-y)*cos(t);
	double dt = atan2(yp-yc,xp-xc) - atan2(y-yc,x-xc);
	while (dt > M_PI) dt -= 2*M_PI;
	while (dt < -M_PI) dt += 2*M_PI;
	double rc = sqrt((x-xc)*(x-xc) + (y-yc)*(y-yc));
	vv = (sc > 0) ? dt*rc : -dt*rc;
	ww = dt;
      }
    else
      {
	vv = sqrt((x-xp)*(x-xp) + (y-yp)*(y-yp));
	ww = 0.0;
      }
    // We evaluate the PDF over the normal distributions based on the noise over the controls
    return nV->evaluate(v-vv) *
      nO->evaluate(w-ww) *
      nT->evaluate(tp-t-ww);
  }
}
