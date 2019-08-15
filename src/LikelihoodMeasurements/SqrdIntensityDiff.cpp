#include "SqrdIntensityDiff.h"


namespace airMCL{

/*This method compares two images using as distance measure the difference
 *of their sum of squared intensities. Both images to compare MUST have the
 *same size.
 *@param img1 The first image to compare
 *@param img2 The second image to compare
 *@return A value that represents the similarity between images. The
 *larger the value the more similar the images are.
*/
double SqrdIntensityDiff::imgMatch(cv::Mat& img1, cv::Mat& img2){

    double result=0;

    /*Get Iterators*/
    cv::Mat_<cv::Vec3b>::iterator it1 = img1.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator it2 = img2.begin<cv::Vec3b>();

    cv::Mat_<cv::Vec3b>::iterator itEnd1 = img1.end<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::iterator itEnd2 = img2.end<cv::Vec3b>();

    /*Process the images*/
    cv::Vec3b diff;
    for( ; it1!=itEnd1 && it2!=itEnd2; ++it1,++it2){
        /*Calculate for each pixel (R1-R2,G1-G2,B1-B2)(R1-R2,G1-G2,B1-B2)T*/
        diff = (*it1) - (*it2);
        result+=diff.dot(diff);
    }

    if(result < 0.1)
        result = 100;
    return 1/result;

}

}
