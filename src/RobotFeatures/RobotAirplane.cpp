#include "RobotAirplane.h"

namespace airMCL
{

    /**
     * 
     * @param map
     * @return 
     */
    cv::Mat Airplane::getObservation
    ( const cv::Mat& map )
    {
        return Observer::observe(map, state, rGaze);
    }


}
