#include "MeanValDistance.h"


namespace airMCL{

    /*This method compares two images using as distance measure the norm of
     *the vector of differences of the means of each color plane.
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    double MeanValDistance::imgMatch(cv::Mat& img1, cv::Mat& img2){

        double result;

        cv::Vec3d means1 = getImgColorMeans(img1);
        cv::Vec3d means2 = getImgColorMeans(img2);

        /*Calculate the norm of the difference between the mean vectors*/
        result = cv::norm(means1,means2, cv::NORM_L2);

        /*Protect against division by zero and big weights*/
        if(result < 0.1)
            result = 0.75;

        return 1/result;
    }

    /*This method gets the mean of each of the color planes of a given
     *image.
     *@param img The image from which to calculate the mean values
     *@return a cv::Vec3d containing the means of the three color planes
    */
    cv::Vec3d MeanValDistance::getImgColorMeans(cv::Mat&img){

        cv::Vec3d result;

        double sumR=0;/*Total pixel values in R channel*/
        double sumG=0;/*Total pixel values in G channel*/
        double sumB=0;/*Total pixel values in B channel*/

        int nr= img.rows; // number of rows
        int nc= img.cols; // number of columns

        double totalPixels = nr*nc;

        /*Loop the whole image*/
        for (int j=0; j<nr; j++) {
            for (int i=0; i<nc; i++) {
                    sumB+=img.at<cv::Vec3b>(j,i)[0];
                    sumG+=img.at<cv::Vec3b>(j,i)[1];
                    sumR+=img.at<cv::Vec3b>(j,i)[2];
              }
        }

        /*Calculate the average values*/
        result[0]=sumB/totalPixels;
        result[1]=sumG/totalPixels;
        result[2]=sumR/totalPixels;

        return result;
    }

}
