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

#include "nd.h"
#include "ndvis.h" // a Stage visualizer for ND

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
    CWanderCtrl ( ARobot* robot )
        : ARobotCtrl ( robot ),
        nd ( 0.2, 0.3, 0.2 ), // robot dimensions in front, behind and to the side of the wheels
        vis ( &nd ),
        mDrivetrain ( NULL ) {
      CLooseStageRobot* looseRobot = ( CLooseStageRobot* ) mRobot;
      looseRobot->findDevice ( mDrivetrain, "position:0" );

      CLooseStageSonar* sonar = NULL;
      looseRobot->findDevice ( sonar, "ranger:0" );

      if ( rapiError->hasError() ) {
        rapiError->print();
        exit ( -1 );
      }

      // insert the visualizer
      mDrivetrain->getStageModel()->AddVisualizer ( &vis, true );

      nd.addRangeFinder ( sonar );
      nd.setGoal ( CPose2d ( -7,0 ) );
    }

    /** Default destructor */
    ~CWanderCtrl() {};

  protected:

    /** autolab-nd*/
    CNd nd;

    /** Stage Visualiser for ND */
    NdVis vis;

    /** Stage position model */
    CLooseStageDrivetrain2dof* mDrivetrain;

    /**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData ( float dt ) {
      nd.update ( mDrivetrain->getTimeStamp(),
                  mDrivetrain->getOdometry()->getPose(),
                  mDrivetrain->getVelocity() );

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
  assert ( robotCtrl );

  return 0; // ok
}


