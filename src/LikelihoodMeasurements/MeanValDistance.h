#ifndef MeanValDistance_h
#define MeanValDistance_h

#include "Likelihood.h"

namespace airMCL{

/*Class that defines an object that compares two images using as distance
measure the norm of the vector of differences of the means of each
color plane*/
class MeanValDistance : virtual public Likelihood {

 public:

    /*This method compares two images using as distance measure the norm of
     *the vector of differences of the means of each color plane
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    double imgMatch(cv::Mat& img1, cv::Mat& img2);

private:

    /*This method gets the mean of each of the color planes of a given
     *image.
     *@param img The image from which to calculate the mean values
     *@return a cv::Vec3d containing the means of the three color planes
    */

    /*NOTE: A separated method was defined to calculate the mean values to
    prevent the case that the images to compare have different sizes*/

    cv::Vec3d getImgColorMeans(cv::Mat&img);
};

}

#endif // MeanValDistance_h
