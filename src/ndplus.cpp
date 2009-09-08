/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: nd.cpp,v 1.8 2009-04-08 22:40:41 jwawerla Exp $
 ***************************************************************************
 *                                                                         *
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
 ***************************************************************************/
#include "ndplus.h"

//-----------------------------------------------------------------------------
CNdPlus::CNdPlus( ABinarySensorArray * bumper, ARangeFinder * ranger,
                  std::string name )
    : CNd( 0.15, 0.15, 0.15, name, 180 )
{
  mBumper = bumper;
  mRanger = ranger;
  addRangeFinder( ranger );
  setVelocityLimits( -INFINITY, INFINITY, -INFINITY, INFINITY );
  setAccelerationLimits( 0.3, INFINITY );
  setEpsilonAngle( D2R( 20.0 ) );
  setEpsilonDistance( 0.1 );
  setAvoidDistance( 0.08 );
  setSafetyDistance( 0.00 );
  setConeSubSampling( 0.0 );
  mEvadeTime = 5.0;
  mEvadeSpeed = -0.2;
  mObstacleTime = 0.0;
  mObstacle = NONE;
  mVRec = 0.0;
  mWRec = 0.0;
}
//-----------------------------------------------------------------------------
CNdPlus::~CNdPlus()
{
}
//-----------------------------------------------------------------------------
CVelocity2d CNdPlus::getRecommendedVelocity()
{
  return CVelocity2d( mVRec, 0.0, mWRec );
}
//-----------------------------------------------------------------------------
void CNdPlus::update( float timestamp, CPose2d pose, CVelocity2d velocity )
{
  float mTimeSinceObstacle = timestamp - mObstacleTime;
  CNd::update( timestamp, pose, velocity );

  if ( mTimeSinceObstacle > mEvadeTime ) {
    mObstacle = NONE;
    mObstacleTime = timestamp;
  }

  if ( isStalled() ) {
    printf( "ND had stalled\n" );
    mObstacle = STALL;
    mObstacleTime = timestamp;
  }
  else if ( mBumper->mBitData[1] ) {
    printf( "Right bumper hit\n" );
    mObstacle = RIGHT;
    mObstacleTime = timestamp;
  }
  else if ( mBumper->mBitData[0] ) {
    printf( "Left bumper hit\n" );
    mObstacle = LEFT;
    mObstacleTime = timestamp;
  }

  if ( mObstacle != NONE ) {
    mVRec = mEvadeSpeed;
    switch ( mObstacle ) {
        case STALL:
        mWRec = 0.0;
        break;
      case RIGHT:
        mWRec = 1.0;
        break;
      case LEFT:
        mWRec = -1.0;
        break;
      default:
        mWRec = 0.0;
        mVRec = 0.0;
        PRT_ERR0( "Unhandled case in obstacle avoidance\n" );
        break;
    }
  }
}
//-----------------------------------------------------------------------------