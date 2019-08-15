#include "HistDiff.h"
#include <iostream>
namespace airMCL{

    HistDiff::HistDiff(){

        /*Initialize variables*/

        comparisonType = CV_COMP_BHATTACHARYYA;
        div=16;

        /*Number of bins for each channel*/
        histSize[0]= histSize[1]= histSize[2]= 256/div;

        /*Define the RGB Range*/
        hranges[0]=0.0;
        hranges[1]=255.0;

        /*All Channel have the same range*/
        ranges[0]=hranges;
        ranges[1]=hranges;
        ranges[2]=hranges;

        /*The three channels*/
        channels[0]=0;
        channels[1]=1;
        channels[2]=2;
    }

    /*This method compares two images using the difference between the images'
     *histograms
     *@param img1 The first image to compare
     *@param img2 The second image to compare
     *@return A value that represents the similarity between images. The
     *larger the value the more similar the images are.
    */
    double HistDiff::imgMatch(cv::Mat& img1, cv::Mat& img2){

        cv::MatND img1H; /*Histogram for the first image*/
        cv::MatND img2H; /*Histogram for the second image*/

        cv::Mat img1R = reduceColor(img1);
        cv::Mat img2R = reduceColor(img2);

        img1H = getColorHistogram(img1R);
        img2H = getColorHistogram(img2R);

        /*TODO: Maybe normalize the histograms*/

        double result = cv::compareHist(img1H,img2H,comparisonType);

        /*Invert result depending on the measurement used*/
        switch(comparisonType){

        /*CV_COMP_CHISQR and CV_COMP_BHATTACHARYYA return 0
        when the images are equal, and a higher value when they are
        different. Need to invert*/

        case CV_COMP_BHATTACHARYYA:
            if(result < 0.00000001)
                result = 0.00000001;
            result = 1-result;
            break;

        case CV_COMP_CHISQR:
            /*Protect against the division by zero and big weights*/
            if(result < 0.1) result = 50;
                result = 1.0/result;
            break;

        /*CV_COMP_INTERSECT and CV_COMP_CORREL are OK, they return a
        "big" value when the images are equal, and a "small" value when
        they are different*/

        default://Do nothing
            break;
        }

        return result;
    }

    /*Sets the type of comparison to be used between histograms.
     *@param cType Type of comparison to be used. Possible values are:
     *CV_COMP_INTERSECT : Bin comparison, keeps the minimum one
     *CV_COMP_CHISQR : Chi Square
     *CV_COMP_CORREL : Cross Correlation
     *CV_COMP_CHISQR : Bhattacharyya measure
     *
    */
    void HistDiff::setComparisonType(int cType){
        comparisonType = cType;
    }

   /*Gets the type of comparison being used
    *@return The type of comparison being used
    */
   int HistDiff::getComparisonType(){
       return comparisonType;
   }

   /*Set the division factor for the color space reduction
    *@param d The reduction factor for the color space. Should be a power of two
    */
   void HistDiff::setDiv(int d){
       div=d;
   }

   /*Get the division factor for the color space reduction
    *@return The reduction factor for the color space. Should be a power of two
    */
   int HistDiff::getDiv(){
       return div;
   }

    /*Method used to reduce the color space of an image
     *@param img The image which color space will be reduced
     *@return A new image based on the one provided, with the color space reduced
    */

   /*This method was adapted from the source code provided by Robert Laganiere from the
    cookbook: Computer Vision Programming using the OpenCV Library. Chapter 4.
    Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name*/
    cv::Mat HistDiff::reduceColor(cv::Mat &img){

        /*Construct the resulting image from the original one*/
        cv::Mat result(img.rows, img.cols, img.type());

        /*Get iterators*/

        /*Iterators on the original image*/
        cv::Mat_<cv::Vec3b>::const_iterator it= img.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itEnd= img.end<cv::Vec3b>();

        /*Iterator on the resulting image*/
        cv::Mat_<cv::Vec3b>::iterator itr = result.begin<cv::Vec3b>();

        /*Do the color reduction*/
        for ( ; it!= itEnd; ++it, ++itr) {
          /*Process each pixel in the image*/
          (*itr)[0]= (*it)[0]/div*div + div/2;
          (*itr)[1]= (*it)[1]/div*div + div/2;
          (*itr)[2]= (*it)[2]/div*div + div/2;
        }

        return result;
    }

    /*Gets a color histogram of a given image. It's stored in a multidimensional Matrix
     *@param img The image from which the color histogram will be calculated
     *@return a MatND containing the color histogram of the image
    */

    /*This method was adapted from the source code provided by Robert Laganiere from the
     cookbook: Computer Vision Programming using the OpenCV Library. Chapter 4.
     Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name*/
    cv::MatND HistDiff::getColorHistogram(const cv::Mat& img){

        cv::MatND hist;

        /*Compute Histogram*/
        cv::calcHist(&img,
            1,			/* Histogram of 1 image only */
            channels,	/* the channel used */
            cv::Mat(),	/* no mask is used */
            hist,		/* The resulting histogram */
            3,			/* It is a 3D histogram */
            histSize,	/* Number of bins */
            ranges		/* Pixel value range */
        );
        return hist;
    }

}
