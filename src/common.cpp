/***************************************************************************
 *   Copyright (C) 2009 by Jens
 *   jwawerla@sfu.ca
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
 * $Log: common.cpp,v $
 * Revision 1.6  2009-04-08 22:40:40  jwawerla
 * Hopefully ND interface issue solved
 *
 * Revision 1.5  2009-04-03 15:10:02  jwawerla
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
 *
 ***************************************************************************/

#include "common.h"
#include <math.h>
#include "utilities.h"

//---------------------------------------------------------------------------
CPose::CPose( float x, float y, float yaw )
{
  mX = x;
  mY = y;
  mYaw = yaw;
}
//---------------------------------------------------------------------------
CPose::CPose(CPose const& pose)
{
  mX = pose.mX;
  mY = pose.mY;
  mYaw = pose.mYaw;
}
//---------------------------------------------------------------------------
CPose::~CPose()
{}
//---------------------------------------------------------------------------
void CPose::operator= (const CPose pose )
{
  mX = pose.mX;
  mY = pose.mY;
  mYaw = pose.mYaw;
}
//---------------------------------------------------------------------------
CPose CPose::operator+ (const CPose pose )
{
  CPose newPose;

  newPose.mX = mX + pose.mX;
  newPose.mY = mY + pose.mY;
  newPose.mYaw = mYaw + pose.mYaw;

  return newPose;
}
//---------------------------------------------------------------------------
bool CPose::operator!= (const CPose pose)
{
  if (mX != pose.mX)
    return true;
  if (mY != pose.mY)
    return true;
  if (mYaw != pose.mYaw)
    return true;

  return false;
}
//---------------------------------------------------------------------------
float CPose::distance( CPose pose )
{
  return sqrt( pow2(pose.mX - mX) + pow2(pose.mY - mY) );
}
//---------------------------------------------------------------------------
float CPose::angleDifference( CPose pose )
{
  return NORMALIZE_ANGLE( mYaw - pose.mYaw );
}
//---------------------------------------------------------------------------
