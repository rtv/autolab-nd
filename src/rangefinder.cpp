/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: rangefinder.cpp,v 1.4 2009-04-03 15:10:02 jwawerla Exp $
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
 * $Log: rangefinder.cpp,v $
 * Revision 1.4  2009-04-03 15:10:02  jwawerla
 * *** empty log message ***
 *
 * Revision 1.3  2009-03-21 02:31:50  jwawerla
 * *** empty log message ***
 *
 * Revision 1.2  2009-03-20 03:05:23  jwawerla
 * dynamic obstacles added to wavefront
 *
 * Revision 1.1.1.1  2009-03-15 03:52:02  jwawerla
 * First commit
 *
 * Revision 1.8  2008/02/07 23:16:24  jwawerla
 * Resizing of rangefinder datastructures
 *
 ***************************************************************************/
#include "rangefinder.h"
#include "error.h"
#include "utilities.h"
#include <stdlib.h>

//---------------------------------------------------------------------------
ARangeFinder::ARangeFinder( int devIndex )
{
  mNumSamples = 0;
  mRangeResolution = 0;
  mMaxRange = 0;
  mMinRange = 0;
  mBeamConeAngle = 0;
  mLFree = -0.1;
  mLOccupied = 0.4;
  mL0 = 0.0;
  mHalfObstacleThickness = 0.6;
  mRelativeBeamPose = NULL;
  mRangeData = NULL;
}
//---------------------------------------------------------------------------
ARangeFinder::~ARangeFinder()
{
}
//------------------------------------------------------------------------------
float ARangeFinder::localInverseRangeSensorModel(float x, float y,
                                                 tPoint2d robotPos,
                                                 float heading)
{
  return localInverseRangeSensorModel(x, y, CPose(robotPos.x, robotPos.y, heading) );
}
//------------------------------------------------------------------------------
float ARangeFinder::localInverseRangeSensorModel(float x, float y,
                                                 CPose robotPose)
{

  float r;            // range from sensor to point [m]
  float phi;          // orientation of beam [rad]
  int k = 0;          // beam index
  float minAngle = INFINITY;
  float a;
  float beamConeAngle;// angle of the beam cone
  float dx;
  float dy;

  // transform global query point location to robot local cs
  dx = x - robotPose.mX;
  dy = y - robotPose.mY;

  // calculate range from sensor to point
  r = sqrt(dx*dx + dy*dy);

  // point out of sensor range ?
  if (r > (mMaxRange + mHalfObstacleThickness) ) {
    return mL0;
  }

  // calculate bearing of query point
  phi = NORMALIZE_ANGLE(atan2(dy, dx) - robotPose.mYaw);

  // make sure we have a minimum beam cone angle
  beamConeAngle = MAX(mBeamConeAngle, D2R(2.5));

  // find the beam index
  // if the beam cone angle is 0, we know it is a laser, therefore
  // we calculate the index of beam that could hit the query point directly
  if (mBeamConeAngle == 0) {
     if (phi < -mFov/2.0)
       k = 0;
     else if (phi > mFov/2.0)
       k = mNumSamples-1;
     else
       k = (int)((phi + mFov/2.0)*mNumSamples/mFov);
  }
  else {
    // we have a sonar or IR sensor, so lets search the beam index, it's only
    // a small number of beams anyway
    for (unsigned int i = 0; i < mNumSamples; i++) {
      a = fabs(phi - mRelativeBeamPose[i].mYaw);
      if ( a < minAngle) {
        minAngle = a;
        k = i;
      }
    } // for
  }

  // query point out of range ?
  if (r > MIN(mMaxRange, mRangeData[k].range + mHalfObstacleThickness) ) {
    return mL0;  // L0
  }

  // bearing of the query point outside of fov ?
  if (fabs(phi - mRelativeBeamPose[k].mYaw ) > beamConeAngle/2.0 ) {
    return mL0;  // L0
  }

  // hit an obstacle ?
  if ( ( mRangeData[k].range < mMaxRange) &&
       ( fabs( r - mRangeData[k].range) < mHalfObstacleThickness) ) {
    return mLOccupied;
  }

  // all clear of obstacles
  return mLFree;
}
//---------------------------------------------------------------------------
float ARangeFinder::getMinRange()
{
  return mMinRange;
}
//---------------------------------------------------------------------------
float ARangeFinder::getMaxRange()
{
  return mMaxRange;
}
//---------------------------------------------------------------------------
void ARangeFinder::print()
{
  printf("Range: sample %03d min %01.2f max %01.2f cone %03.1f", mNumSamples,
          mMinRange, mMaxRange, R2D(mBeamConeAngle) );
  printf("\n");
}
//---------------------------------------------------------------------------
