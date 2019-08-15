#ifndef DISTRIBUTIONS_H
#define	DISTRIBUTIONS_H
#include <opencv.hpp>
#include <iostream>
#include <chrono>
#include <random>
#include <cmath>

namespace airMCL
{
  /** Univariate Distributions.
   *  Based on JB Hayet Java implementations. We acknowlege JB-Hayet for the contribution to this proyect.
   **/
  
  /*
   * Abstract class modeling a generic implementation of a univariate probability distribution.
   */
  class univariateDistribution 
  {
    // Method to generate a value from the distribution
    virtual double generate()=0;
    // Method to evaluate the PDF of the distribution over a value
    virtual double evaluate(double)=0;
  };
  
  /*
   * Concrete class modeling a Normal Distribution.
   */
  class normalUnivariateDistribution : univariateDistribution
  {
  public:
    // Constructor of the class
    normalUnivariateDistribution(double m, double v): mean(m), variance(v)
    {
      // Get a time-based seed
      unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      generator = new std::default_random_engine(seed);
      distribution = new std::normal_distribution<double>(m,sqrt(v));

      constant = 1.0/sqrt(2*M_PI*variance);
    }

    ~normalUnivariateDistribution()
    {
      delete distribution;
      delete generator;
    }

    // Implementation of abstract methods

    // Generation of a normal distributed random value using Box-Muller method
    double generate()
    {
      return (*distribution)(*generator);
    }

    // Evaluation of the density over a value
    double evaluate(double value)
    {
      return constant*exp(-0.5*pow(value-mean,2.0)/variance);
    }

  private:
    double mean;
    double variance;
    double constant; // The constant of normalization
    // std Random engine generator
    std::default_random_engine *generator;
    std::normal_distribution<double> *distribution;
  };

  /*
   * Concrete class modeling a Triangular Distribution.
   */
  class triangularUnivariateDistribution : univariateDistribution
  {
  public:
    // Constructor of the class
    triangularUnivariateDistribution(double m, double v): mean(m), variance(v)
    {
      // We get the Standard Deviation
      sd = sqrt(variance);
      distribution = new std::uniform_int_distribution<int>(0, RAND_MAX);

      c1 = 1.0/sqrt(6*variance);
      c2 = 1.0/(6*variance);
    }

    ~triangularUnivariateDistribution()
    {
      delete distribution;
    }

    // Implementation of abstract methods
  
    // Generation of a triangular distributed random value
    double generate()
    {
      // Limits of the uniform distribution
      double max = sd;
      double min = -sd;
      // We generate two uniform values
      double n1 = /*std::rand()*/(*distribution)(generator)*(max-min+1)/(double(RAND_MAX)) + min;
      double n2 = /*std::rand()*/(*distribution)(generator)*(max-min+1)/(double(RAND_MAX)) + min;
      // Evaluation of the density
      return sqrt(6)*(n1+n2)/2;
    }
    
    // Evalution of the density over a value
    double evaluate(double value)
    {
      return std::max(0.0, c1 - fabs(value-mean)*c2);
    }

  private:
    double mean;
    double variance;
    double sd; // Standard Deviation
    double c1;
    double c2;
    // std Random engine generator
    std::default_random_engine generator;
    std::uniform_int_distribution<int> *distribution;
  };

    /*
     * Generic distribution
     */
    class genericDistribution
    {
        public:
          // function to generate a sample from the distribution
          virtual cv::Mat generate()=0;
          
          // function to get real random values
          static int rdtsc()
          { __asm__ volatile("rdtsc"); }
          
          /**
           * Method that generates a random number in a specified range
           * @param min Inferior limit of range
           * @param max Superior limit of range
           * @return Number generated in the range
           */
          static double rand(double min, double max)
          {
             // Random generator seed
             srand(rdtsc());
             
             // Generating a random number between min and max
             return ((std::rand()/(double(RAND_MAX)))*(max-min) + min);
          }
          
          /**
           * Method that generates a random number from a normal distribution
           * using Box-Muller method.
           * @param mean Mean of the normal distribution
           * @param variance Variance of the normal distribution
           * @return Random number generated with the parameters
           */
          double rnormal(double mean, double variance)
          {
	    double n1 = std::rand()/(double(RAND_MAX));
	    double n2 = std::rand()/(double(RAND_MAX));
	    
	    return sqrt(-2.0*variance*log(n1))*cos(2*M_PI*n2) + mean; 
	  }
      
          /**
	   * Method that evaluates a value over a normal PDF
	   * @param value Value to evaluate
	   * @param mean Mean of the normal distribution
	   * @param variance Variance of the normal distribution
	   * @return Evaluation of the PDF
	   */
          double rnevaluate(double value, double mean, double variance)
	  {
	    return exp(-0.5*pow(value-mean,2.0)/variance)/sqrt(2*M_PI*variance);
	  }
    };
    
    
    /*
     * Uniform distribution: implements genericdistribution
     */
    class uniform : public genericDistribution
    {
        public:
            
            // Alias for multidimentional ranges
            typedef std::vector<std::pair<double, double> > MultiRange;
            
            /**
             * Constructor by dimention and a set of ranges for each dimention
             * @param dim Dimention of distribution
             * @param range Vector containing a pair of number (range) for each dimention
             */
            uniform(int dim, const MultiRange& range)
            : d(dim), ranges(range)
            { }

            /*
             * Method to generate a sample in its ranges
             */
            cv::Mat generate()
            {
                // sample of doubles
                cv::Mat sample = cv::Mat(d, 1, CV_64FC1);
                
                for(register int i=0; i<d; i++)
                    sample.at<double>(i, 0) = rand( ranges[i].first, ranges[i].second );
                
                return sample;
            }

        private:

            // Dimension of distribution
            int d;

            // Range for each dimention
            MultiRange ranges;
    };    
    
    
    /*
     * Normal distribution: implements genericdistribution
     */
    class normal : public genericDistribution
    {
        public:
            
            /**
             * Constructor by dimention, mean values and coavariance
             * @param d Dimention of distribution
             * @param mean Mean of the distribution
             * @param cov Covariance of distribution
             */
            normal(int d, const cv::Mat& mean, const cv::Mat& cov)
            : d(d), mean(mean), Cov(cov)
            { }
            
            /*
             * Given the fact that to evaluate a multivariate normal
             * we need the Cholesky factorization of the covariance matrix,
             * and for the motion model only need a single value, we pospose
             * the implementation of this method.
             */
            cv::Mat generate()
            {
                return cv::Mat::zeros(2,2, CV_64FC1);
            }
            
        private:
        
            // Dimention of distribution
            int d;
            
            // Mean of distribution
            cv::Mat mean;
            
            // Covariance of distribution
            cv::Mat Cov;            
    };
    
}


#endif	/* DISTRIBUTIONS_H */

