/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 * 2018-10-18
 */


#include "TMEbraitenberg/include/TMEbraitenbergController.h"
#include "WorldModels/RobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include <vector>
#include <iostream>

using namespace std;

// Load readable sensor names
#define NB_SENSORS 12 // should be coherent with gRobotSpecsImageFilename value read from the property file.
#include "Utilities/Sensorbelt.h"
vector<double> vect(12,0.0);
Point2d  old_pos;

TMEbraitenbergController::TMEbraitenbergController( RobotWorldModel *__wm ) : Controller ( __wm )
{
    old_pos = getPosition();
    
	
    if ( _wm->_cameraSensorsNb != NB_SENSORS )
    {
        std::cerr << "[CRITICAL] This project assumes robot specifications with " << NB_SENSORS << " sensors (provided: " << _wm->_cameraSensorsNb << " sensors). STOP.\n";
        exit(-1);
    }
}

TMEbraitenbergController::~TMEbraitenbergController()
{
    // nothing to do.
}

void TMEbraitenbergController::reset()
{
    // nothing to do.
}

void TMEbraitenbergController::step()
{
     Point2d cur_pos;

     setTranslation( sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0) );

     double rotation=0.0;

     for ( int i = 0 ; i != NB_SENSORS ; i++ ) // also possible to check each sensor separatly using SENSOR_L, SENSOR_R, etc. (check Utilities/Sensorbelt.h)
        {
            double dist = getDistanceAt(i);

            
            if ( getWallAt(i) <= 1 )
            {

		vect[i]=1-dist;
            }
            else{
		vect[i]=0.0;
	    }
            if(i<=3 || i==4 || i==10 || i==11)
		rotation+=vect[i];
	    if((i>=5 && i<=8) || i==9)
		rotation+=-vect[i];
        }
	setRotation(rotation);

	cur_pos=getPosition();

	if(old_pos.x==cur_pos.x  && old_pos.y == cur_pos.y){
		std::cout << "------------------------------random\n";
		setRotation( 0.1 - (double)(random01()*0.2));
		setTranslation( sin( ( ( getDistanceAt(SENSOR_FFL) + getDistanceAt(SENSOR_FFR) ) / 2.0 )* M_PI / 2.0) );
	}
	old_pos=cur_pos;
}

