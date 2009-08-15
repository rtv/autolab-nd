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
#include "nd2_alg.h"
#include "printerror.h"
#include "utilities.h"
#include <string.h>
#include <math.h>


//-----------------------------------------------------------------------------
CNd::CNd ( float frontDim, float backDim, float sideDim, std::string robotName )
{
  mRobotName = robotName;
  mFrontDim = frontDim;
  mBackDim = backDim;
  mSideDim = sideDim;

  mState = AT_GOAL;
  mAngularSubSamples = D2R(1.0);
  mSensorList.clear();
  mReadingIndex = 0;
  mFgReadingBufferInitialized = false;
  mRobotRadius = 0.4;
  mSafetyDist = 0.1;
  mAvoidDist = 0.4;
  mDistEps = 0.6;
  mAngleEps = D2R ( 10.0 );
  mVMax = 0.45;
  mVMin = 0.02;
  mWMax = D2R ( 45.0 );
  mWMin = D2R ( 0.0 );
  mVCmd = 0.0;
  mWCmd = 0.0;
  mCurrentTime = 0.0f;
  mFgWaiting = false;
  mFgTurningInPlace = false;
  mFgStalled = false;
  mFgCrossedPathNormal = false;
  mFgAtGoal = true;
  mRotateStuckTime = 10.0;  // [s]
  mTranslateStuckTime = 2.0;
  mTranslateStuckDist = 0.25;
  mTranslateStuckAngle = D2R ( 20.0 );
  mObstacles.longitud = 0;
  mVDotMax = 0.75;
  mWDotMax = 0.75;
  mCurrentTime = 0.0;
  mRotateMinError = 0.0;
  mRotateStartTime = 0.0;

  mFrontAvoidBox.rect.setCoordinates(0.12, -0.25, 0.4, 0.25);
  mBackAvoidBox.rect.setCoordinates(-0.4, -0.25, -0.12, 0.25);

  mLeftAvoidBox.rect.setCoordinates ( -0.41, -0.3, 0.41, 0.0 );
  mRightAvoidBox.rect.setCoordinates ( -0.41,  0.0, 0.41, 0.3 );

  // Fill in the ND's parameter structure

  // Rectangular geometry = 1, round = 0
  //mNDparam.geometryRect = 1;
  mNDparam.geometryRect = 0;
  // Distance from the wheel to the front
  mNDparam.front = frontDim + mSafetyDist;
  // Distance from the wheel to the back
  mNDparam.back = backDim +  mSafetyDist;
  // Distance from the wheel to the left side (note robot is symetric, therefore
  // no need for right side
  mNDparam.left = sideDim + mSafetyDist;

  mNDparam.R = 0.3F;   // radius of robot, used only if geometryRect = 0

  mNDparam.holonomic = 0;  // Non holonomic vehicle

  mNDparam.vlmax = mVMax;
  mNDparam.vamax = mWMax;

  mNDparam.almax = mVDotMax;  // maximal translational acceleration
  mNDparam.aamax = mWDotMax;  // maximal rotational acceleration

  mNDparam.dsmax = mAvoidDist; // Security distance
  mNDparam.dsmin = mNDparam.dsmax / 4.0F;
  //mNDparam.enlarge = NDparametros.dsmin/2.0F;
  mNDparam.enlarge = mNDparam.dsmin * 0.2F;

  mNDparam.discontinuity = 1.5 * mNDparam.left;  // Discontinuity

  mNDparam.T = 0.1F;  // Sample rate of the SICK

  // Pass the structure to ND for initialization
  InicializarND ( &mNDparam );

  reset();
  mInfo.regiones.longitud = 0;
}
//-----------------------------------------------------------------------------
CNd::~CNd()
{
}
//-----------------------------------------------------------------------------
CVelocity2d CNd::getRecommendedVelocity()
{
  return CVelocity2d ( mVCmd, 0.0, mWCmd );
}
//-----------------------------------------------------------------------------
void CNd::setSafetyDistance ( float dist )
{
  mSafetyDist = dist;
  // Distance from the wheel to the front
  mNDparam.front = mFrontDim + mSafetyDist;
  // Distance from the wheel to the back
  mNDparam.back = mBackDim +  mSafetyDist;
  // Distance from the wheel to the left side (note robot is symetric, therefore
  // no need for right side
  mNDparam.left = mSideDim + mSafetyDist;

  // Pass the structure to ND for initialization
  InicializarND ( &mNDparam );
}
//-----------------------------------------------------------------------------
void CNd::setAvoidDistance ( float dist )
{
  mAvoidDist = dist;

  mNDparam.dsmax = mAvoidDist; // Security distance
  // Pass the structure to ND for initialization
  InicializarND ( &mNDparam );
}
//-----------------------------------------------------------------------------
void CNd::reset()
{
  mFgWaiting = false;
  mFgTurningInPlace = false;
  mFgStalled = false;
  mFgAtGoal = false;
}
//-----------------------------------------------------------------------------
void CNd::setEpsilonAngle ( float angle )
{
  mAngleEps = angle;
}
//-----------------------------------------------------------------------------
void CNd::setEpsilonDistance ( float dist )
{
  mDistEps = dist;
}
//-----------------------------------------------------------------------------
void CNd::addRangeFinder ( ARangeFinder* sensor )
{
  mSensorList.push_back ( sensor );
}
//-----------------------------------------------------------------------------
bool CNd::atGoal()
{
  return mFgAtGoal;
}
//-----------------------------------------------------------------------------
bool CNd::isStalled()
{
  return mFgStalled;
}
//-----------------------------------------------------------------------------
bool CNd::hasActiveGoal()
{
  return mFgActiveGoal;
}
//-----------------------------------------------------------------------------
void CNd::setGoal ( CPose2d goal )
{
  mGoal = goal;
  mFgActiveGoal = true;
  mFgAtGoal = false;
  mFgStalled = false;
  mFgTurningInPlace = false;
  mTranslateStartTime = mCurrentTime;
  mLastRobotPose = mRobotPose;
  mState = NORMAL;
}
//-----------------------------------------------------------------------------
void CNd::setConeSubSampling( float samplesPerRad )
{
  mAngularSubSamples = samplesPerRad;
}
//-----------------------------------------------------------------------------
void CNd::processSensors()
{
  ARangeFinder* rf;
  float rx, ry;
  float range;
  float globalSensorX, globalSensorY, globalSensorAngle;
  float cosR, sinR;
  float maxRange;
  int subsamples;
  float angle;
  float beamConeAngle;
  CPoint2d point;

  mFrontAvoidBox.fgObstacle = false;
  mRightAvoidBox.fgObstacle = false;
  mLeftAvoidBox.fgObstacle = false;
  mBackAvoidBox.fgObstacle = false;

  // sin and cosin of robots headings
  sinR = sin ( mRobotPose.mYaw );
  cosR = cos ( mRobotPose.mYaw );

  mFgRobotRadiusPenetrated = false;
  mReadingIndex = 0;

  if ( mSensorList.empty() ) {
    PRT_ERR1 ( "%s: No range finders available", mRobotName.c_str() );
    mObstacles.longitud = 0;
    return;
  }

  for ( unsigned int s = 0; s < mSensorList.size(); s++ ) {
    rf = mSensorList[s];
    beamConeAngle = rf->getBeamConeAngle();
    maxRange = rf->getMaxRange();
    for ( unsigned int i = 0; i < rf->getNumSamples(); i++ ) {

      range = rf->mRangeData[i].range;

      if ( range < mRobotRadius )
        mFgRobotRadiusPenetrated = true;

      // calculate global sensor coordinates
      rx = rf->mRelativeBeamPose[i].mX;
      ry = rf->mRelativeBeamPose[i].mY;

      globalSensorX = mRobotPose.mX + rx * cosR - ry * sinR;
      globalSensorY = mRobotPose.mY + rx * sinR + ry * cosR;
      globalSensorAngle = normalizeAngle ( mRobotPose.mYaw +
                                            rf->mRelativeBeamPose[i].mYaw );

      // is this a laser type device with practically no beam cone ?
      if (beamConeAngle == 0) {
        // convert to cartesian coords, in global coordinate system
        mObstacles.punto[mReadingIndex].x = globalSensorX + range *
                                          cos ( globalSensorAngle );
        mObstacles.punto[mReadingIndex].y = globalSensorY + range *
                                          sin ( globalSensorAngle );
        mReadingIndex ++;
      }
      // or a device with a beam cone like a sonar or ir sensor
      else {
        subsamples = ceil(0.5 * beamConeAngle / mAngularSubSamples);
        for (int n = -subsamples; n < subsamples; n++) {
          // convert to cartesian coords, in global coordinate system
          angle = normalizeAngle(globalSensorAngle + n * mAngularSubSamples );
          mObstacles.punto[mReadingIndex].x = globalSensorX + range *
                                            cos ( angle );
          mObstacles.punto[mReadingIndex].y = globalSensorY + range *
                                            sin ( angle );
          mReadingIndex ++;
        }

      }

      //***************************************
      // check boxes for obstacles
      // the boxes are defined in robot CS so are the range readings
      point.mX =  rx + cos ( rf->mRelativeBeamPose[i].mYaw ) * range;
      point.mY =  ry + sin ( rf->mRelativeBeamPose[i].mYaw ) * range;

      if ( mFrontAvoidBox.rect.isInside ( point ) ) {
        mFrontAvoidBox.fgObstacle = true; // not clear
      }
      if ( mRightAvoidBox.rect.isInside ( point ) ) {
        mRightAvoidBox.fgObstacle = true; // not clear
      }
      if ( mLeftAvoidBox.rect.isInside ( point ) ) {
        mLeftAvoidBox.fgObstacle = true; // not clear
      }
      if ( mBackAvoidBox.rect.isInside ( point ) ) {
        mBackAvoidBox.fgObstacle = true; // not clear
      }

      if ( mReadingIndex >= MAX_POINTS_SCENARIO ) {
        mReadingIndex = 0;
        mFgReadingBufferInitialized = true;
      }
    }
    if ( mFgReadingBufferInitialized )
      mObstacles.longitud = MAX_POINTS_SCENARIO;
    else
      mObstacles.longitud = mReadingIndex;
  }
}
//-----------------------------------------------------------------------------
/*
void CNd::update ( float timestamp, CPose2d robotPose,
                   CVelocity2d robotVelocity )
{
  tState prevState = mState;
  TVelocities *cmdVel;
  TCoordenadas goal;
  TInfoMovimiento motionData;
  float goalDX, goalDA;

  // do we have a goal?
  if ( !mFgActiveGoal )
    return;

  // set robot pose in GLOBAL CS
  motionData.SR1.posicion.x = robotPose.mX;
  motionData.SR1.posicion.y = robotPose.mY;
  motionData.SR1.orientacion = robotPose.mYaw;
  // set velocity
  motionData.velocidades.v = robotVelocity.mXDot;
  motionData.velocidades.w = robotVelocity.mYawDot;
  motionData.velocidades.v_theta = 0.0f;


  mRobotPose = robotPose;

  processSensors();

  // The current odometric goal
  goal.x = mGoal.mX;
  goal.y = mGoal.mY;

  // calculate distance to goal and angular offset
  goalDX = mGoal.distance ( mRobotPose );
  goalDA = mGoal.angleDifference ( mRobotPose );

  // check if we have reached the goal pose
  if ( ( goalDX <= mDistEps ) &&
       ( goalDA <= mAngleEps ) ) {
    mState = AT_GOAL;
  }

  // Call ND and get a velocity command
  cmdVel = IterarND ( goal,            // goal
                      mDistEps,        // goal tolerance
                      &motionData,     // current velocity of the robot
                      &mObstacles,     // list of the obstacle points in global cs
                      &mInfo );        // ND puts internal data in here

  if ( cmdVel == NULL ) {
    PRT_WARN1 ( "%s: Some thing went wrong, ND returned NULL", mRobotName.c_str() );
    mVCmd = 0.0f;
    mWCmd = 0.0f;
    return;
  }

  switch ( mState ) {
    case NORMAL:
      mFgStalled = false;
      mFgTurningInPlace = false;
      mVCmd = cmdVel->v;
      mWCmd = cmdVel->w;

      if ( checkStalled() )
        mState = PRE_STALLED;

      if ( goalDX < mDistEps )
        mState = ALIGNING;
      break;

    case ALIGNING:
      if ( not mFgTurningInPlace ) {
        // first time; cache the time and current heading error
        mRotateStartTime = mCurrentTime;
        mRotateMinError = fabs ( goalDA );
        mFgTurningInPlace = true;
        PRT_MSG1 ( 9, "%s: Turning in place", mRobotName.c_str() );
      }
      else {
        // Are we making progress?
        if ( fabs ( goalDA ) < mRotateMinError ) {
          // yes; reset the time
          mRotateStartTime = mCurrentTime;
          mRotateMinError = fabs ( goalDA );
        }
        else {
          // no; have we run out of time?
          if ( ( mCurrentTime - mRotateStartTime ) > mRotateStuckTime ) {
            mState = PRE_STALLED;
          }
        }
      }
      printf ( "%s %f \n", mRobotName.c_str(), R2D ( goalDA ) );

      if ( not mFgRobotRadiusPenetrated ) {
        mWCmd = mWMax * max ( fabs ( goalDA ) / PI, 0.1 ) * SIGN ( goalDA );
        mVCmd = 0.0;
      }
      else {
        mVCmd = 0.0;
        mWCmd = 0.0;
        PRT_MSG1 ( 1, "%s: cannot turn, obstacle too close\n", mRobotName.c_str() );
      }
      break;

    case AT_GOAL:
      mVCmd = 0.0f;
      mWCmd = 0.0f;
      mFgAtGoal = true;
      mFgActiveGoal = false;
      mFgTurningInPlace = false;
      break;


    case PRE_STALLED:
      if (mFgStateChanged && ( fabs ( mInfo.angle ) < HALF_PI ) ) {
        mState = STALLED;
      }
      else {
        mVCmd = 0.0f;
        mWCmd = -mWMax * SIGN(mInfo.angle);
      }
      break;

    case STALLED:
      mVCmd = 0.0f;
      mWCmd = 0.0f;
      mFgStalled = true;
      mFgTurningInPlace = false;
      mFgActiveGoal = false;

      break;
  } // switch

  if (mState != prevState )
    mFgStateChanged = true;
  else
    mFgStateChanged = false;

  mVCmd = threshold ( mVCmd, mVMin, mVMax );
}
*/
//-----------------------------------------------------------------------------
void CNd::checkCrossedPathNormal()
{
  float dist;
  float pathNormalDist;
  float pathX;
  float pathY;

  pathX = cos ( mGoal.mYaw );
  pathY = sin ( mGoal.mYaw );
  dist = mGoal.mX * pathX + mGoal.mY * pathY;

  pathNormalDist = mRobotPose.mX * pathX +
                   mRobotPose.mY * pathY - dist;

  //printf("pathNormalDist %f \n", pathNormalDist);
  if ( ( mPathNormalDist < 0 ) && ( pathNormalDist >= 0 ) )
    mFgCrossedPathNormal = true;

  mPathNormalDist = pathNormalDist;
}
//-----------------------------------------------------------------------------
bool CNd::checkStalled()
{
  // Have we moved far enough?
  float oDx = hypot ( mRobotPose.mX - mLastRobotPose.mX,
                      mRobotPose.mY - mLastRobotPose.mY );
  float oDa = normalizeAngle ( mRobotPose.mYaw - mLastRobotPose.mYaw );

  if ( ( oDx > mTranslateStuckDist ) ||
       ( fabs ( oDa ) > mTranslateStuckAngle ) ) {
    mLastRobotPose = mRobotPose;
    mTranslateStartTime = mCurrentTime;
  }
  else {
    // Has it been long enough?
    if ( ( mCurrentTime - mTranslateStartTime ) > mTranslateStuckTime ) {
      PRT_MSG1 ( 6, "%s: ran out of time trying to get to goal",
                 mRobotName.c_str() );
      return true;
    }
  }
  return false;
}
//-----------------------------------------------------------------------------

void CNd::update ( float timestamp, CPose2d robotPose,
                   CVelocity2d robotVelocity )
{
  TVelocities *cmdVel = NULL;
  TCoordenadas goal;
  TInfoMovimiento motionData;
  float gDx, gDa;

  // already up to data
  if ( mCurrentTime == timestamp)
    return;

  // increment time
  mCurrentTime = timestamp;
  mRobotPose = robotPose;

  processSensors();

  // are we waiting for a stall to clear?
  if ( mFgWaiting )
    return;

  // do we have a goal?
  if ( !mFgActiveGoal ) {
    return;
  }

  // set robot pose in GLOBAL CS
  motionData.SR1.posicion.x = mRobotPose.mX;
  motionData.SR1.posicion.y = mRobotPose.mY;
  motionData.SR1.orientacion = mRobotPose.mYaw;
  // set velocity
  motionData.velocidades.v = robotVelocity.mXDot;
  motionData.velocidades.w = robotVelocity.mYawDot;
  motionData.velocidades.v_theta = 0.0f;

  // are we at the goal?
  gDx = hypot ( mGoal.mX - mRobotPose.mX,
                mGoal.mY - mRobotPose.mY );

  //gDa = angleDiff ( mGoal.mYaw, mRobotPose.mYaw );
  gDa = normalizeAngle ( mGoal.mYaw - mRobotPose.mYaw );

  // Are we at the goal yet ??
  if ( ( gDx < mDistEps ) && ( fabs ( gDa ) < mAngleEps ) ) {
    mFgActiveGoal = false;
    mVCmd = 0.0f;
    mWCmd = 0.0f;
    PRT_MSG1 ( 6, "%s: At goal", mRobotName.c_str() );
    mFgAtGoal = true;
    return;
  }
  else {
    // are we close enough in distance?
    if ( ( gDx < mDistEps ) || ( mFgTurningInPlace ) ) {
      PRT_MSG1 ( 9, "%s: Turning in place", mRobotName.c_str() );
      //printf ( "%s %f \n", mRobotName.c_str(), R2D ( gDa ) );

      if ( not mFgRobotRadiusPenetrated ) {
        mWCmd = mWMax * max ( fabs ( gDa ) / PI, 0.1 ) * sign ( gDa );
        mVCmd = 0.0;
      }
      else {
        mVCmd = 0.0;
        mWCmd = 0.0;
        //printf ( "%s: cannot turn, obstacle too close\n", mRobotName.c_str() );
      }
      // To make the robot turn (safely) to the goal orientation, we'll
      // give it a fake goal that is in the right direction, and just
      // ignore the translational velocity.
      goal.x = mRobotPose.mX + 10.0 * cos ( mGoal.mYaw );
      goal.y = mRobotPose.mY + 10.0 * sin ( mGoal.mYaw );

      if ( not mFgTurningInPlace ) {
        // first time; cache the time and current heading error
        mRotateStartTime = mCurrentTime;
        mRotateMinError = fabs ( gDa );
        mFgTurningInPlace = true;
      }
      else {
        // Are we making progress?
        if ( fabs ( gDa ) < mRotateMinError ) {
          // yes; reset the time
          mRotateStartTime = mCurrentTime;
          mRotateMinError = fabs ( gDa );
        }
        else {
          // no; have we run out of time?
          if ( ( mCurrentTime - mRotateStartTime ) > mRotateStuckTime ) {
            PRT_MSG1 ( 6, "%s: Ran out of time trying to attain goal heading",
                       mRobotName.c_str() );
            mVCmd = 0.0f;
            mWCmd = 0.0f;
            mFgStalled = true;
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
      float oDa = normalizeAngle ( mRobotPose.mYaw - mLastRobotPose.mYaw );

      if ( ( oDx > mTranslateStuckDist ) ||
           ( fabs ( oDa ) > mTranslateStuckAngle ) ) {
        mLastRobotPose = mRobotPose;
        mTranslateStartTime = mCurrentTime;
      }
      else {
        // Has it been long enough?
        if ( ( mCurrentTime - mTranslateStartTime ) > mTranslateStuckTime ) {
          PRT_MSG1 ( 6, "%s: ran out of time trying to get to goal",
                     mRobotName.c_str() );
          mVCmd = 0.0f;
          mWCmd = 0.0f;
          mFgStalled = true;
          mFgActiveGoal = false;
          return;
        }
      }

// The current odometric goal
      goal.x = mGoal.mX;
      goal.y = mGoal.mY;

      cmdVel = IterarND ( goal,            // goal
                          mDistEps,        // goal tolerance
                          &motionData,     // current velocity of the robot
                          &mObstacles,     // list of the obstacle points in global cs
                          &mInfo );        //ND leaves internal data in here

      if ( !cmdVel ) {
        // Emergency stop
        mVCmd = 0.0f;
        mWCmd = 0.0f;
        mFgStalled = true;
        mFgActiveGoal = false;
        PRT_MSG1 ( 6, "%s: Emergency stop", mRobotName.c_str() );
        return;
      }
      else {

        mFgStalled = false;
        mVCmd = cmdVel->v;
        mWCmd = cmdVel->w;
      }
    } // far away loop

    if ( !mVCmd && !mWCmd ) {
// ND is done, yet we didn't detect that we reached the goal.  How odd.
      mVCmd = 0.0f;
      mWCmd = 0.0f;
      mFgStalled = true;
      mFgActiveGoal = false;
      PRT_MSG1 ( 9, "%s: ND failed to reach goal ?!", mRobotName.c_str() );
      return;
    }
    else {
      mVCmd = threshold ( mVCmd, mVMin, mVMax );
    }
  } // if (!mVCmd && !mWCmd)
}

//-----------------------------------------------------------------------------
float CNd::angleDiff ( float a, float b )
{
  float d1, d2;
  a = normalizeAngle ( a );
  b = normalizeAngle ( b );
  d1 = a - b;
  d2 = 2 * M_PI - fabs ( d1 );

  if ( d1 > 0 )
    d2 *= -1.0;

  if ( fabs ( d1 ) < fabs ( d2 ) ) {
    return ( d1 );
  }
  else {
    return ( d2 );
  }

}
//-----------------------------------------------------------------------------
float CNd::threshold ( float v, float vMin, float vMax )
{
  if ( isAboutZero ( v ) )
    return ( v );
  else
    if ( v > 0.0 ) {
      v = min ( v, vMax );
      v = max ( v, vMin );
      return ( v );
    }
    else {
      v = max ( v, -vMax );
      v = min ( v, -vMin );
      return ( v );
    }
}
//-----------------------------------------------------------------------------


