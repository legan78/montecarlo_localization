/* 
 * File:   Airplane.h
 * Author: angel
 *
 * Created on 2 de abril de 2013, 11:52 AM
 */

#ifndef AIRPLANE_H
#define	AIRPLANE_H
#include "Observer.h"

namespace airMCL
{
    class Airplane
    {
    public:
        
        /**
         * Default constructor.
         */
        Airplane()
        : state(State())
        {  }
        
        /**
         * Copy constructor.
         * @param other Other airplane robot
         */
        Airplane(const Airplane& other)
        : state(other.state)
        {
            for(register int i=0; i<4; i++)
               rGaze[i] = other.rGaze[i];
        }
        
        /**
         * Constructor using a given state.
         * @param st State for the robot
         */
        Airplane(const State& st)
        : state(st)
        { }
        
        /**
         * Destructor
         */
        ~Airplane()
        { }
        
        /**
         * Method to obtain an observation of the robot, i.e. an image of what
         * the robot sees of the map
         * @param map Image of the map where the robot is being localized
         * @return An image of what the robot sees
         */
        cv::Mat getObservation(const cv::Mat& map);
        
        /**
         * 
         * @return 
         */
        const cv::Point2f* getObsROIPoints() const
        { return rGaze; }
        
        
        /**
         * 
         * @return 
         */
        const State& getState() const
        { return state;  }
        
        
        void setState(const State& st)
        { state = st;}
        
    private:
        
        cv::Point2f rGaze[4];       // Points of rectangle of observation (what robot sees)
        State       state;          // Current state of the robot
        
        // Here goes the robot deadreckoning model
        
    };
}


#endif	/* AIRPLANE_H */

