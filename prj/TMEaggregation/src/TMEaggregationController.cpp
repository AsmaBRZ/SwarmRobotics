/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEaggregation/include/TMEaggregationController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include <map>

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"

TMEaggregationController::TMEaggregationController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEaggregationController::~TMEaggregationController()
{
    // nothing to do.
}

void TMEaggregationController::reset()
{
    // nothing to do.
}

void TMEaggregationController::step()
{   
    for ( int i = 0 ; i != NB_SENSORS ; i++ ) {

	 double dist = getDistanceAt(i);
            //std::cout << "\t\t[" << i << "] ";
            if(getWallAt(i) != 1) 
            {   
                int robotId = getRobotIdAt(i);
                if ( robotId != -1 )
                {
                    std::cout << "[dist=" << dist << "] Robot #" << robotId << "\n\t\t      relative orientation wrt robot "<<getId()<<" : " << getRobotRelativeOrientationAt( i )*180.0 << "°\n";
                    
                    // accessing the target robot's controller
                    // can be useful if additional methods have been implemented (e.g. communication)
                    TutorialController* targetRobotController = dynamic_cast<TutorialController*>(getRobotControllerAt(i));
                    std::cout << "\t\t      [double-check] target robot's id really is #" << targetRobotController->getId() << "\n"; // example of use
                    
                    sendMessage(targetRobotController,"Ping");
                    
                }
                else
                {
                    std::cout << "[dist=" << dist << "] Nothing\n";
                }
            }
    }
}
