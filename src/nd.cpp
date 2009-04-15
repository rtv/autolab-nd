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
 ***************************************************************************
 * $Log: nd.cpp,v $
 * Revision 1.8  2009-04-08 22:40:41  jwawerla
 * Hopefully ND interface issue solved
 *
 * Revision 1.7  2009-04-03 15:10:02  jwawerla
 * *** empty log message ***
 *
 * Revision 1.6  2009-03-29 00:54:27  jwawerla
 * Replan ctrl seems to work now
 *
 * Revision 1.5  2009-03-21 02:31:50  jwawerla
 * *** empty log message ***
 *
 * Revision 1.4  2009-03-20 03:05:23  jwawerla
 * dynamic obstacles added to wavefront
 *
 * Revision 1.3  2009-03-17 03:11:30  jwawerla
 * Another day no glory
 *
 * Revision 1.2  2009-03-16 14:27:18  jwawerla
 * robots still get stuck
 *
 * Revision 1.1.1.1  2009-03-15 03:52:02  jwawerla
 * First commit
 *
 * Revision 1.2  2008/03/17 23:58:40  jwawerla
 * Added logwrite support
 *
 *
 ***************************************************************************/
#include "nd.h"
#include "string.h"

/** Dimension of robot in front of wheels [m] */
const float FRONT_DIMENSION = 0.4;
/** Dimension of robot in back of wheels [m] */
const float BACK_DIMENSION = 0.3;
/** Dimension to the side of the center of rotation [m] */
const float SIDE_DIMENSION = 0.2;

//---------------------------------------------------------------------------
CNd::CNd ( const char* robotName )
{
  if ( robotName == NULL )
    snprintf ( mRobotName, 20, "noname" );
  else
    strncpy ( mRobotName, robotName, 20 );

  mNumSensors = 0;
  mSafetyDist = 0.1;
  mAvoidDist = 0.6;
  mDistEps = 0.6;
  mAngleEps = D2R ( 10 );
  mVMax = 0.45;
  mVMin = 0.02;
  mWMax = D2R ( 45.0 );
  mWMin = D2R ( 0.0 );
  mVCmd = 0;
  mWCmd = 0;
  mFgWaiting = false;
  mFgWaitOnStall = false;
  mTurningInPlace = false;
  mFgStall = false;
  mFgAtGoal = true;
  mRotateStuckTime = 5.0;
  mTranslateStuckTime = 2.0;
  mTranslateStuckDist = 0.25;
  mTranslateStuckAngle = D2R ( 20 );
  mObstacles.longitud = 0;
  mVDotMax = 0.75;
  mWDotMax = 0.75;
  // Fill in the ND's parameter structure

  // Rectangular geometry
  mNDparam.geometryRect = 1;
  // Distance from the wheel to the front
  mNDparam.front = FRONT_DIMENSION + mSafetyDist;
  // Distance from the wheel to the back
  mNDparam.back = BACK_DIMENSION +  mSafetyDist;
  // Distance from the wheel to the left side (note robot is symetric, therefore
  // no need for right sight
  mNDparam.left = SIDE_DIMENSION + mSafetyDist;

  mNDparam.R = 0.2F;   // Not used

  mNDparam.holonomic = 0;  // Non holonomic vehicle

  mNDparam.vlmax = mVMax;
  mNDparam.vamax = mWMax;

  mNDparam.almax = mVDotMax;  // maximal translational acceleration
  mNDparam.aamax = mWDotMax;  // maximal rotational acceleration

  mNDparam.dsmax = mAvoidDist; // Security distance
  mNDparam.dsmin = mNDparam.dsmax / 4.0F;
  //NDparametros.enlarge = NDparametros.dsmin/2.0F;
  mNDparam.enlarge = mNDparam.dsmin * 0.2F;

  mNDparam.discontinuity = mNDparam.left;  // Discontinuity (check whether it fits)

  mNDparam.T = 0.1F;  // Sample rate of the SICK

  // Pass the structure to ND for initialization
  InicializarND ( &mNDparam );
  // set current driving direction to forward
  mCurrentDir = 1;

  mTauWTurnInPlace = 0.01;
  mTauW = 0.06;

}
//---------------------------------------------------------------------------
CNd::~CNd()
{
}
//---------------------------------------------------------------------------
void CNd::reset()
{
  mFgWaiting = false;
  mFgWaitOnStall = false;
  mTurningInPlace = false;
  mFgStall = false;
  mFgAtGoal = false;
}
//---------------------------------------------------------------------------
void CNd::setEpsilonAngle ( float angle )
{
  mAngleEps = angle;
}
//---------------------------------------------------------------------------
void CNd::setEpsilonDistance ( float dist )
{
  mDistEps = dist;
}
//---------------------------------------------------------------------------
void CNd::addRangeFinder ( ARangeFinder* sensor )
{
  if ( mNumSensors < MAX_ND_SENSORS ) {
    mSensorList[mNumSensors] = sensor;
    mNumSensors++;
  }
}
//---------------------------------------------------------------------------
bool CNd::atGoal()
{
  return mFgAtGoal;
}
//---------------------------------------------------------------------------
bool CNd::isStalled()
{
  return mFgStall;
}
//---------------------------------------------------------------------------
bool CNd::hasActiveGoal()
{
  return mFgActiveGoal;
}
//---------------------------------------------------------------------------
void CNd::setGoal ( CPose goal )
{
  mGoal = goal;
  mFgActiveGoal = true;
  mFgAtGoal = false;
  mFgStall = false;
  mTurningInPlace = false;
  mTranslateStartTime = timeStamp();
  mLastRobotPose = mRobotPose;
}
//---------------------------------------------------------------------------
void CNd::processSensors()
{
  float x, y;
  float rx, ry;
  float cosR, sinR;
  float cosA, sinA;
  float maxRange;
  int idx = 0;

  sinR = sin ( mRobotPose.mYaw );
  cosR = cos ( mRobotPose.mYaw );

  for ( int s = 0; s < mNumSensors; s++ ) {
    maxRange = mSensorList[s]->getMaxRange();
    for ( unsigned int i = 0; i < mSensorList[s]->mNumSamples; i++ ) {

      if ( mSensorList[s]->mRangeData[i].range < maxRange ) {
        sinA = sin ( mSensorList[s]->mRelativeBeamPose[i].mYaw );
        cosA = cos ( mSensorList[s]->mRelativeBeamPose[i].mYaw );

        // convert to cartesian coords, in the laser's frame
        x = mSensorList[s]->mRangeData[i].range * cosA;
        y = mSensorList[s]->mRangeData[i].range * sinA;

        rx = x + mSensorList[s]->mRelativeBeamPose[i].mX;
        ry = y + mSensorList[s]->mRelativeBeamPose[i].mY;

        // convert to the odometric frame and add to the obstacle list
        mObstacles.punto[idx].x = ( mRobotPose.mX + rx * cosR - ry * sinR );
        mObstacles.punto[idx].y = ( mRobotPose.mY + rx * sinR + ry * cosR );
      }
      else {
        mObstacles.punto[idx].x = 0;
        mObstacles.punto[idx].y = 0;
      }
      idx ++;
    }
    mObstacles.longitud = idx;
  }
}
//---------------------------------------------------------------------------
void CNd::update ( CPose robotPose )
{
  TVelocities *cmdVel;
  TCoordenadas goal;
  TInfoMovimiento pose;
  float gDx, gDa;

  // are we waiting for a stall to clear?
  if ( mFgWaiting )
    return;

  // do we have a goal?
  if ( !mFgActiveGoal )
    return;

  // The robot's current odometric pose
  pose.SR1.posicion.x = robotPose.mX;
  pose.SR1.posicion.y = robotPose.mY;
  pose.SR1.orientacion = robotPose.mYaw;

  mRobotPose = robotPose;

  processSensors();

  if ( mObstacles.longitud == 0 ) {
    PRT_ERR1 ( "%s: No sensor data available, did you register a range finder ?", mRobotName );
    return;
  }

  // TODO: put a smarter check earlier
  //assert( mObstacles.longitud <= MAX_POINTS_SCENARIO );

  // are we at the goal?
  gDx = hypot ( mGoal.mX - mRobotPose.mX,
                mGoal.mY - mRobotPose.mY );

  gDa = angleDiff ( mGoal.mYaw, mRobotPose.mYaw );

  // Are we there?
  if ( ( gDx < mDistEps ) && ( fabs ( gDa ) < mAngleEps ) ) {
    mFgActiveGoal = false;
    mVCmd = 0;
    mWCmd = 0;
    PRT_MSG1 ( 6, "%s: At goal", mRobotName );
    mFgAtGoal = true;
    return;
  }
  else {
    // are we close enough in distance?
    if ( ( gDx < mDistEps ) || ( mTurningInPlace ) ) {
      PRT_MSG1 ( 9, "%s: Turning in place", mRobotName );
      // To make the robot turn (safely) to the goal orientation, we'll
      // give it a fake goal that is in the right direction, and just
      // ignore the translational velocity.
      goal.x = mRobotPose.mX + 10.0 * cos ( mGoal.mYaw );
      goal.y = mRobotPose.mY + 10.0 * sin ( mGoal.mYaw );

      // In case we went backward to get here, reverse direction so that we
      // can attain the goal heading
      setDirection ( 1 );

      cmdVel = IterarND ( goal,
                          mDistEps,
                          &pose,
                          &mObstacles,
                          NULL );
      if ( !cmdVel ) {
        // Emergency stop
        mVCmd = 0;
        mWCmd = 0;
        mFgStall = true;
        mFgActiveGoal = false;
        PRT_MSG1 ( 6, "%s: Emergency stop", mRobotName );
        return;
      }
      else {
        mFgStall = false;
      }
      // we are turning in place, so ignore translational speed command
      mVCmd = 0.0;
      //mWCmd = mWCmd + mTauWTurnInPlace / mNDparam.T  * ( cmdVel->w - mWCmd );
      mWCmd = cmdVel->w;

      if ( !mTurningInPlace ) {
        // first time; cache the time and current heading error
        mRotateStartTime = timeStamp();
        mRotateMinError = fabs ( gDa );
        mTurningInPlace = true;
      }
      else {
        // Are we making progress?
        if ( fabs ( gDa ) < mRotateMinError ) {
          // yes; reset the time
          mRotateStartTime = timeStamp();
          mRotateMinError = fabs ( gDa );
        }
        else {
          // no; have we run out of time?
          if ( ( timeStamp() - mRotateStartTime ) > mRotateStuckTime ) {
            PRT_MSG1 ( 6, "%s: Ran out of time trying to attain goal heading", mRobotName );
            mVCmd = 0;
            mWCmd = 0;
            mFgStall = true;
            mFgActiveGoal = false;
            return;
          }
        }
      }
    }

    // we're far away; execute the normal ND loop
    else {
      // Have we moved far enough?
      float oDx = hypot ( mRobotPose.mX - mLastRobotPose.mX,
                          mRobotPose.mY - mLastRobotPose.mY );
      float oDa = angleDiff ( mRobotPose.mYaw,
                              mLastRobotPose.mYaw );

      if ( ( oDx > mTranslateStuckDist ) ||
           ( fabs ( oDa ) > mTranslateStuckAngle ) ) {
        mLastRobotPose = mRobotPose;
        mTranslateStartTime = timeStamp();
      }
      else {
        // Has it been long enough?
        float t;
        t = timeStamp();

        if ( ( t - mTranslateStartTime ) > mTranslateStuckTime ) {
          PRT_MSG1 ( 6, "%s: ran out of time trying to get to goal", mRobotName );
          mVCmd = 0;
          mWCmd = 0;
          mFgStall = true;
          mFgActiveGoal = false;
          return;
        }
      }

      // The current odometric goal
      goal.x = mGoal.mX;
      goal.y = mGoal.mY;

      // Were we asked to go backward?
      if ( mDir < 0 ) {
        // Trick the ND by telling it that the robot is pointing the
        // opposite direction
        pose.SR1.orientacion = NORMALIZE_ANGLE ( pose.SR1.orientacion + M_PI );
        // Also reverse the robot's geometry (it may be asymmetric
        // front-to-back)
        setDirection ( -1 );
      }
      else
        setDirection ( 1 );

      cmdVel = IterarND ( goal,
                          mDistEps,
                          &pose,
                          &mObstacles,
                          NULL );

      if ( !cmdVel ) {
        // Emergency stop
        mVCmd = 0;
        mWCmd = 0;
        mFgStall = true;
        mFgActiveGoal = false;
        PRT_MSG1 ( 6, "%s: Emergency stop", mRobotName );
        return;
      }
      else {

        mFgStall = false;
        mVCmd = cmdVel->v;
        mWCmd = cmdVel->w;
      }
      //mWCmd = mWCmd + mTauW / mNDparam.T  * ( cmdVel->w - mWCmd );
    } // far away loop

    if ( !mVCmd && !mWCmd ) {
      // ND is done, yet we didn't detect that we reached the goal.  How
      // odd.
      mVCmd = 0;
      mWCmd = 0;
      mFgStall = true;
      mFgActiveGoal = false;
      PRT_MSG1 ( 9, "%s: ND failed to reach goal ?!", mRobotName );
      return;
    }
    else {
      mVCmd = threshold ( mVCmd, mVMin, mVMax );

      if ( !mVCmd )
        mWCmd = threshold ( mWCmd, mWMin, mWMax );
      // Were we asked to go backward?
      if ( mDir < 0 ) {
        // reverse the commanded x velocity
        mVCmd = -mVCmd;
      }
    } // if (!mVCmd && !mWCmd)
  }
}

//---------------------------------------------------------------------------
float CNd::angleDiff ( float a, float b )
{
  float d1, d2;
  a = NORMALIZE_ANGLE ( a );
  b = NORMALIZE_ANGLE ( b );
  d1 = a - b;
  d2 = 2 * M_PI - fabs ( d1 );

  if ( d1 > 0 )
    d2 *= -1.0;

  if ( fabs ( d1 ) < fabs ( d2 ) ) {
    //printf ( "angleDiff(%f %f) = %f (%f) \n", R2D ( a ), R2D ( b ), R2D ( d1 ), R2D ( d2 ) );
    return ( d1 );
  }
  else {
    //printf ( "angleDiff(%f %f) = %f (%f) \n", R2D ( a ), R2D ( b ), R2D ( d2 ), R2D ( d1 ) );
    return ( d2 );
  }

}

//---------------------------------------------------------------------------
void CNd::setDirection ( int dir )
{
  if ( dir == mCurrentDir )
    return;

  if ( dir > 0 ) {
    // Distance to the front
    mNDparam.front = FRONT_DIMENSION + mSafetyDist;

    // Distance to the back
    mNDparam.back = BACK_DIMENSION + mSafetyDist;
    InicializarND ( &mNDparam );
  }
  else {
    // Distance to the front
    mNDparam.front = BACK_DIMENSION + mSafetyDist;
    // Distance to the back
    mNDparam.back = FRONT_DIMENSION + mSafetyDist;
    InicializarND ( &mNDparam );
  }

  mCurrentDir = dir;
}

//---------------------------------------------------------------------------
float CNd::threshold ( float v, float vMin, float vMax )
{
  if ( v == 0.0 )
    return ( v );
  else
    if ( v > 0.0 ) {
      v = MIN ( v, vMax );
      v = MAX ( v, vMin );
      return ( v );
    }
    else {
      v = MAX ( v, -vMax );
      v = MIN ( v, -vMin );
      return ( v );
    }
}
//-----------------------------------------------------------------------------


