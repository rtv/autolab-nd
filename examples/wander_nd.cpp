/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca), Richard Vaughan (vaughan@sfu.ca)*
 * $Id: $
 ***************************************************************************
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 **************************************************************************/

#include <RapiLooseStage>
using namespace Rapi;

#include "../src/nd.h"

/**
 * Example robot controller, based on the wander example from Stage
 * @author Jens Wawerla, Richard Vaughan
 */
class CWanderCtrl : public ARobotCtrl
{
 public:
  /**
	* Default constructor
	* @param robot to control
	*/
 CWanderCtrl(ARobot* robot)
	: ARobotCtrl ( robot ),
	  nd( "example robot"),
	  mDrivetrain(NULL)
  {
	 CLooseStageRobot* looseRobot = (CLooseStageRobot*)mRobot;	 	 
	 looseRobot->findDevice ( mDrivetrain, "position:0" );
	 
	 CLooseStageLaser* laser = NULL;
	 looseRobot->findDevice ( laser, "laser:0" );
	 
	 if ( rapiError->hasError() ) {
		rapiError->print();
		exit ( -1 );
	 }

	 nd.addRangeFinder( laser );
	 nd.setGoal( CPose2d(6,6) );
  }
 
  /** Default destructor */
  ~CWanderCtrl(){};

 protected:
  
  /** autolab-nd*/
  CNd nd;

  /** Stage position model */
  CLooseStageDrivetrain2dof* mDrivetrain;

  /**
	* Update controller for the current time step
	* @param dt time since last upate [s]
	*/
  void updateData(float dt)
  {
	 nd.update( mDrivetrain->getTimeStamp(),
					mDrivetrain->getOdometry()->getPose()  );
	 
	 mDrivetrain->setVelocityCmd ( nd.getRecommendedVelocity() );
	 
	 //printf( "v: %.2f  w: %.2f\n", timestamp, vel.mVX, vel.mYawDot );
  }
};


extern "C" int Init ( Stg::Model* mod )
{

  // init general stuff
  ErrorInit ( 1, false );
  initRandomNumberGenerator();

  // create robot and its controller
  ARobotCtrl* robotCtrl = new CWanderCtrl ( new CLooseStageRobot ( mod ) );

  return 0; // ok
}


