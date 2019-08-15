#ifndef HistDiff_h
#define HistDiff_h

#include <opencv2/imgproc/imgproc.hpp>
#include "Likelihood.h"


namespace airMCL{

/*Class that defines an object that compares two images using as distance
measure the difference between the images' histograms*/
class HistDiff : virtual public Likelihood {

 public:

    /*Default constructor*/
    HistDiff();

    /*This method compares two images using the difference between the images'
     *histograms
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    double imgMatch(cv::Mat& img1, cv::Mat& img2);

    /*Sets the type of comparison to be used between histograms.
     *@param cType Type of comparison to be used. Possible values are:
     *CV_COMP_INTERSECT : Bin comparison, keeps the minimum one
     *CV_COMP_CHISQR : Chi Square
     *CV_COMP_CORREL : Cross Correlation
     *CV_COMP_BHATTACHARYYA : Bhattacharyya measure
     *
    */
    void setComparisonType(int cType);

   /*Gets the type of comparison being used
    *@return The type of comparison being used
    */
   int getComparisonType();


   /*Set the division factor for the color space reduction
    *@param d The reduction factor for the color space. Should be a power of two
    */
   void setDiv(int d);

   /*Get the division factor for the color space reduction
    *@return The reduction factor for the color space. Should be a power of two
    */
   int getDiv();


 private:
    /*Method used to reduce the color space of an image
     *@param img The image which color space will be reduced
     *@return A new image based on the one provided, with the color space reduced
    */
    cv::Mat reduceColor(cv::Mat &img);

    /*Gets a color histogram of a given image. It's stored in a multidimensional Matrix
     *@param img The image from which the color histogram will be calculated
     *@return a MatND containing the color histogram of the image
    */
    cv::MatND getColorHistogram(const cv::Mat& img);

  /*Class Members*/

  /*Variable that holds the type of comparison to be used */
  int comparisonType;
  /*Variable that holds the reduction factor for color space reduction*/
  int div;

  /*Auxiliary variables for histogram calculation*/

  /*Number of bins for each channel*/
  int histSize[3];
  /*RGB Range*/
  float hranges[2];
  /*Ranges for each channel*/
  const float* ranges[3];
  /*The channels of the histogram*/
  int channels[3];

};

}

#endif // HistDiff_h
