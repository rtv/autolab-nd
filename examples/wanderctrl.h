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

#include "RapiLooseStage"

using namespace Rapi;
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
	: ARobotCtrl ( robot )
  {
	 CLooseStageRobot* looseRobot;
	 
	 mRobot = robot;
	 mAvoidcount = 0;
	 mCounter = 0;
	 
	 looseRobot = ( CLooseStageRobot* ) mRobot;
	 looseRobot->findDevice ( mDrivetrain, "position:0" );
	 looseRobot->findDevice ( mPowerPack, "powerpack:0" );
	 looseRobot->findDevice ( mLaser, "laser:0" );
	 looseRobot->findDevice ( mTextDisplay, "textdisplay:0" );
	 looseRobot->findDevice ( mFiducial, "model:0.fiducial:0" );
	 if ( rapiError->hasError() ) {
		rapiError->print();
		exit ( -1 );
	 }
	 mRobot->mVariableMonitor.addVar(&mAvoidcount, "avoid count");
	 mRobot->mVariableMonitor.addVar(&mFgAvoid, "avoiding");
  }
 
  /** Default destructor */
  ~CWanderCtrl(){};

 protected:
  
  /** Stage position model */
  CLooseStageDrivetrain2dof* mDrivetrain;
  /** Stage laser model */
  CLooseStageLaser* mLaser;
  /** Stage power pack */
  CLooseStagePowerPack* mPowerPack;
  /** Text display */
  CLooseStageTextDisplay* mTextDisplay;
  /** Fiducial Finder */
  CLooseStageFiducialFinder* mFiducial;
  /** Count avoid time steps */
  int mAvoidcount;
  /** Just for test purpose */
  bool mFgAvoid;
  /** Just for test purpose */
  int mCounter;

  /**
	* Update controller for the current time step
	* @param dt time since last upate [s]
	*/
  void updateData(float dt)
  {
	 bool obstruction = false;
	 bool stop = false;
	 double minleft = 1e6;
	 double minright = 1e6;
		
	 // find the closest distance to the left and right and check if
	 // there's anything in front
		
	 mCounter++;
	 rprintf("hello %d \n", mCounter);
		
	 for ( uint32_t i = 0; i < mLaser->getNumSamples(); i++ ) {
		  
		if ( ( i > ( mLaser->getNumSamples() /3 ) )
			  && ( i < ( mLaser->getNumSamples() - ( mLaser->getNumSamples() /3 ) ) )
			  && mLaser->mRangeData[i].range < MINFRONTDISTANCE ) {
		  PRT_MSG0 ( 4, "obstruction!" );
		  obstruction = true;
		}
		  
		if ( mLaser->mRangeData[i].range < STOPDIST ) {
		  PRT_MSG0 ( 4, "stopping!" );
		  stop = true;
		}
		  
		if ( i > mLaser->getNumSamples() /2 )
		  minleft = MIN ( minleft, mLaser->mRangeData[i].range );
		else
		  minright = MIN ( minright, mLaser->mRangeData[i].range );
	 }
	 PRT_MSG1 ( 4, "minleft %.3f \n", minleft );
	 PRT_MSG1 ( 4, "minright %.3f\n ", minright );


	 if ( obstruction || stop || ( mAvoidcount>0 ) ) {
		PRT_MSG1 ( 4, "Avoid %d\n", mAvoidcount );

		mDrivetrain->setTranslationalSpeedCmd ( stop ? 0.0 : AVOIDSPEED );

		/* once we start avoiding, select a turn direction and stick
			with it for a few iterations */
		if ( mAvoidcount < 1 ) {
		  PRT_MSG0 ( 4,"Avoid START" );
		  mAvoidcount = random() % AVOIDDURATION + AVOIDDURATION;

		  if ( minleft < minright ) {
			 mDrivetrain->setRotationalSpeedCmd ( -AVOIDTURN );
			 PRT_MSG1 ( 4,"turning right %.2f\n", -AVOIDTURN );
		  } else {
			 mDrivetrain->setRotationalSpeedCmd ( +AVOIDTURN );
			 PRT_MSG1 ( 4, "turning left %2f\n", +AVOIDTURN );
		  }
		}

		mAvoidcount--;
	 } else {
		PRT_MSG0 ( 4, "Cruise" );

		mAvoidcount = 0;
		mDrivetrain->setVelocityCmd ( CRUISESPEED, 0.0 );
	 }

	 if ( mDrivetrain->isStalled() ) {
		mDrivetrain->setVelocityCmd ( CRUISESPEED, 0.0 );
	 }
  }
};


extern "C" int Init ( Stg::Model* mod )
{
  CGui* gui = CGui::getInstance(0, NULL);

  CLooseStageRobot* robot;
  ARobotCtrl* robotCtrl;

  // init general stuff
  ErrorInit ( 1, false );
  initRandomNumberGenerator();

  // create robot and its controller
  robot = new CLooseStageRobot ( mod );
  robotCtrl = new CWanderCtrl ( robot );

  gui->registerRobot(robot);
  return 0; // ok
}


