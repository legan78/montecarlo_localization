#ifndef Likelihood_h
#define Likelihood_h

#include <opencv2/core/core.hpp>


namespace airMCL{

/*Interface that all image similarity measures shall implement*/
class Likelihood {

public:

    /*This method compares two images using some distance measure to be defined
     *by the implementers of this interface.
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    virtual double imgMatch(cv::Mat& img1, cv::Mat& img2)  = 0;

    // virtual destructor for interface
    virtual ~Likelihood() { }
};

}

#endif // Likelihood_h
