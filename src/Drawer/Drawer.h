/* 
 * File:   Drawer.h
 * Author: angel
 *
 * Created on 2 de abril de 2013, 12:58 PM
 */

#ifndef DRAWER_H
#define	DRAWER_H

#include "../RobotFeatures/RobotAirplane.h"
#include "../ParticleFilter/particle.h"

namespace airMCL
{
    /**
     * Class to draw things for the localization problem.
     */
    class Drawer
    {
       public:
           
         /**
          * MEthod to draw a rectangle of what the robot is looking at.
          * @param robot Robot from which the meassure is generated.
          * @param map Image from where the observation is going to be taken.
          * @param color Color to draw the rectangle, red by default.
          */
         static void drawRobotObs
         ( const Airplane& robot, cv::Mat& map, 
           cv::Scalar color =  cv::Scalar(255,0,255) )
         {
             const cv::Point2f* rPoints = robot.getObsROIPoints();
             // Drawing rectangle through points
             for(int i=0; i<4; i++)
                 cv::line(map, rPoints[i], rPoints[(i+1)%4], color, 5, CV_AA);
         }
         
         
         /**
          * Method to draw the current robot state on the image map
          * @param st Current state of the robot to draw
          * @param map Image map where to draw the robot state
          */
         static void drawRobotState
         ( const State& st, cv::Mat& map,
           cv::Scalar color = cv::Scalar( 0, 0, 255 ) )
         {
             // Drawing a circle corresponding to the particle
             cv::circle(map, cv::Point2f(st.x, st.y), 5, color, 5);
             // Drawing a line corresponding to the particle direction
             cv::line(map, cv::Point2f(st.x, st.y),
                           cv::Point2f(st.x+ cos(st.theta)*30 , 
                                       st.y+ sin(st.theta)*30),
                           color, 1, CV_AA);
         }
         
         /**
          * Method to draw the ellipse of covariance on the position estimated
          * @param St Matrix of covariance
          * @param st Current state estimated of the robot
          * @param map Image map where is being localizating
          * @param color Color to draw the ellipse
          */
         static void drawCovariance
         ( const cv::Mat& St, const State& st, cv::Mat& map,
           cv::Scalar color = cv::Scalar( 0,128,255 ) )
         {
             cv::Mat values, vects;
             double pi=3.1416;
             
             cv::eigen(St, values, vects);
             double l1 = sqrt(values.at<double>(0,0));
             double l2 = sqrt(values.at<double>(1,0));
             
             double theta =  std::atan( vects.at<double>(0,0)/vects.at<double>(1,0) );
             
             cv::ellipse( map, 
                          cv::Point(st.x, st.y), 
                          cv::Size(3.0*l1, 3.0*l2), 
                          theta*180.0/pi, 
                          0, 360,
                          color ,3);
         }
         
         
         
         /**
          * Method to draw the particles of the particle filter.
          * @param set Set of current particles.
          * @param map Image where to draw the particles.
          */
         static void drawParticles
         ( const std::vector<Particle>& set, cv::Mat& map, 
           cv::Scalar color = cv::Scalar( 255,255,0 ) ) // Cyan color
         {
             unsigned int i=0;

             for( i=0; i<set.size(); i++)
             {
                // Drawing a circle corresponding to the particle
                cv::circle(map, cv::Point2f(set[i].state().x, set[i].state().y), 2, color, 2);
                // Drawing a line corresponding to the particle direction
                cv::line(map, cv::Point2f(set[i].state().x, set[i].state().y),
                              cv::Point2f(set[i].state().x+ cos(set[i].state().theta)*10 , 
                                          set[i].state().y+ sin(set[i].state().theta)*10),
                              color, 1, CV_AA);
             }

         }
         
    };
}


#endif	/* DRAWER_H */

