/* 
 * 
 * 
 * 
 * 
 * 
 */

#ifndef STATE_H
#define	STATE_H

#include <opencv.hpp>
#include <cmath>

namespace airMCL
{
    struct State
    {
        State( )
        :x(51),y(51),theta(0)
        { }
        
        State(double x, double y, double theta)
        : x(x), y(y), theta(theta) 
        { }
        
        State(const State& st)
        :x(st.x), y(st.y), theta(st.theta)
        { }
        
        State& operator=(const State& st)
        {
            x = st.x;
            y = st.y;
            theta = st.theta;
            return *this;
        }
        
        State operator+(const State& st) const
        {
            State rt = *this;
            rt.x += st.x;
            rt.y += st.y;
            rt.theta += st.theta;
            return rt;
        }

        State operator-(const State& st) const
        {
            State rt = *this;
            rt.x -= st.x;
            rt.y -= st.y;
            rt.theta -= st.theta;
            return rt;
        }

        State operator*(const double &d) const
        {
            State rt = *this;
            rt.x *= d;
            rt.y *= d;
            rt.theta *= d;
            return rt;
        }

        State operator/(const double &d) const
        {
            State rt = *this;
            rt.x /= d;
            rt.y /= d;
            rt.theta /= d;
            return rt;
        }

        bool operator>(const double &d)
        {
            return (x > d && y > d && theta > d);
        }
		
        double x;                   // x position in the map
        double y;                   // y position in the map
        double theta;               // rotation
    };    
    
}

#endif	/* STATE_H */

