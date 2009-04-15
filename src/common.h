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
 * $Log: common.h,v $
 * Revision 1.5  2009-03-29 00:54:27  jwawerla
 * Replan ctrl seems to work now
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
 **************************************************************************/

#ifndef COMMON_H
#define COMMON_H



/** Type definition of  a 2d point */
typedef struct {
  float x;
  float y;
} tPoint2d;

/** Type definition of a range data */
typedef struct {
  /** Range for sensor to object or max sensor range [m] */
  double range;
  /** Intensity of the reflection 0.0 to 1.0 */
  double reflectance;
} tRangeData;

/** 2d - Pose definition */
class CPose
{
  public:
    CPose ( float x = 0.0, float y = 0.0, float yaw = 0.0 );
    /**
    * Constructor
    * @param pose to copy pose from
    */
    CPose ( CPose const& pose );
    /** Default destructor */
    ~CPose();
    /**
     * Gets the euclidian distance to a given pose
     * @param pose to get distance to
     * @return distance [m]
     */
    float distance( CPose pose );
    /**
     * Gets the angular difference between this pose and the given pose
     * @return [rad]
     */
    float angleDifference( CPose pose );
    /** Overloaded = operator */
    void operator= (const CPose pose );
    /** Overloaded + operator */
    CPose operator+ (const CPose pose );
    /** Overloaded != operator */
    bool operator!= (const CPose pose);
    /** x position [m] */
    float mX;
    /** y position [m] */
    float mY;
    /** yaw angle [rad] */
    float mYaw;
};
#endif
