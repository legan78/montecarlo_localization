#ifndef SqrdIntensityDiff_h
#define SqrdIntensityDiff_h

#include "Likelihood.h"

namespace airMCL{

/*Class that defines an object that compares two images using as distance
measure the difference of their sum of squared intensities*/
class SqrdIntensityDiff : public Likelihood {

 public:

    /*This method compares two images using as distance measure the difference
     *of their sum of squared intensities. Both images to compare MUST have the
     *same size.
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    double imgMatch(cv::Mat& img1, cv::Mat& img2);
};

}
#endif // SqrdIntensityDiff_h
