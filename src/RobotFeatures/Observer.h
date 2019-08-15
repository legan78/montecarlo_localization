/* 
 * 
 * 
 * 
 * 
 * 
 */

#ifndef OBSERVER_H
#define	OBSERVER_H


#include "State.h"

// Size of the image measurement
#define WIDTH_MEASURE   51
#define HEIGHT_MEASURE  51

namespace airMCL
{
    class Observer {
      public:
            
        /**
         * Method to obtain an image of observation given the robot current state.
         * The method extracts a square image ROI of the image map in the direction
         * stated by the state direction.
         * @param map Image map from wich the robot is getting an observation.
         * @param state Current state of the robot.
         * @return Square image ROI containg pixels taken from the observation position.
         */
        static cv::Mat observe
        ( const cv::Mat& map, const State& state, cv::Point2f rPoints[] )
        {
            double pi = 3.1416;
 
            cv::RotatedRect rect2;
            double c= (51.0)*(1.0/sqrt(2.0));
            
            cv::Mat roi=map(cv::Range(int(state.y)-c, int(state.y)+c), cv::Range(int(state.x)-c,int(state.x)+c));
            
            rect2.center.x = roi.cols/2.0 + (state.x-int(state.x));
            rect2.center.y = roi.cols/2.0 + (state.y-int(state.y));
            rect2.angle    = state.theta*180.0/pi;
            rect2.size     = cv::Size2f(WIDTH_MEASURE, HEIGHT_MEASURE);
            
            // cv::Mat to rotate image and obtain the ROI
            cv::Mat rotated2, ROI2;
            
            // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
            if (rect2.angle < -45.) 
            {
               rect2.angle += 90.0;
               std::swap(rect2.size.width, rect2.size.height);
            }

            // Rotation matrix
            cv::Mat M2 = cv::getRotationMatrix2D(rect2.center, rect2.angle, 1.0);
            // Perform the affine transformation ( rotation of image )
            cv::warpAffine(roi, rotated2, M2, roi.size(), cv::INTER_CUBIC);
            // Crop the resulting image
            cv::getRectSubPix(rotated2, rect2.size, rect2.center, ROI2);
            // Obtainig a rectangle in image coordinates of what the robot sees
            rect2.points(rPoints);
            
            for(int i=0; i<4; i++)
            {
                rPoints[i].x+=(state.x-rect2.center.x);
                rPoints[i].y+=(state.y-rect2.center.y);
            }
            
//=======================================================            
            return ROI2;
        }
    };
}



#endif	/* OBSERVER_H */

