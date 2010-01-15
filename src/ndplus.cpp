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
                  std::string name, unsigned int maxPoints )
    : CNd( 0.15, 0.15, 0.15, name, maxPoints )
{
  mBumper = bumper;
  mRanger = ranger;
  addRangeFinder( ranger );
  setVelocityLimits( -INFINITY, INFINITY, -INFINITY, INFINITY );
  setAccelerationLimits( 0.5, INFINITY );
  setEpsilonAngle( D2R( 20.0 ) );
  setEpsilonDistance( 0.1 );
  setAvoidDistance( 0.05 );
  setSafetyDistance( 0.00 );
  setConeSubSampling( 0.0 );
  mObstacleTime = 0.0;
  mObstacle = NONE;
  mVRec = 0.0;
  mWRec = 0.0;

  mBackupTime = 0.1;
  mBackupSpeed = -0.1;
  mTurnTime = 0.2;
  mTurnThreshold = D2R( 90.0 );
  mTurnRate = D2R( 90.0 ) / 1.0 ;
  srand( time( NULL ) );
  mRearThreshold = 0.25;
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
  CVelocity2d ndVelo = CNd::getRecommendedVelocity();
  mVRec = ndVelo.mXDot;
  mWRec = ndVelo.mYawDot;
  //mRearRange = mRanger->mRangeData[3].range;

  // finish evading obstacle
  if ( mTimeSinceObstacle > (mBackupTime + mTurnTime) ) {
    mObstacle = NONE;
    mObstacleTime = timestamp;
  }

  // check for obstacles
  if ( isStalled() || mBumper->isAnyTriggered() ) {
    mObstacleTime = timestamp;
    mRandom = 2.0 * ( ((double) rand() ) / RAND_MAX );
    if ( isStalled() )
      mObstacle = STALL;
    else if ( mBumper->mBitData[1] && mBumper->mBitData[0] )
      mObstacle = BOTH;
    else if ( mBumper->mBitData[1] )
      mObstacle = RIGHT;
    else if ( mBumper->mBitData[0] )
      mObstacle = LEFT;
  }

  // take evasive action: backup then turn away
  if ( mObstacle != NONE ) {
    if( mTimeSinceObstacle < mBackupTime ) {
      mVRec = mBackupSpeed;
      mWRec = 0.0;
    }
    else {
      mVRec = 0.0;
      mWRec = ( mObstacle == RIGHT ) ? -mTurnRate : mTurnRate;
    }
    //if( mRearRange < mRearThreshold )
    //  mVRec = 0.0;
  }
}
//-----------------------------------------------------------------------------
