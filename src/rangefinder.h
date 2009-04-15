/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: rangefinder.h,v 1.3 2009-04-03 15:10:02 jwawerla Exp $
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
 ***************************************************************************
 * $Log: rangefinder.h,v $
 * Revision 1.3  2009-04-03 15:10:02  jwawerla
 * *** empty log message ***
 *
 * Revision 1.2  2009-03-20 03:05:23  jwawerla
 * dynamic obstacles added to wavefront
 *
 * Revision 1.1.1.1  2009-03-15 03:52:02  jwawerla
 * First commit
 *
 * Revision 1.8  2008/03/19 00:23:43  gumstix
 * Minor modifications and bugfixing after CB test
 *
 *
 ***************************************************************************/
#ifndef ARANGEFINDER_H
#define ARANGEFINDER_H

#include <stdio.h>
#include "common.h"

/**
 * Abstract base class for all range finder devices such as lasers,
 * IR sensors or sonars
 * @author Jens Wawerla <jwawerla@sfu.ca>
 * @version 0.1 - 11/2007
 */
class ARangeFinder
{
  public:
    /** Default destructor */
    virtual ~ARangeFinder();
    /**
     * This method gets called by the framework every step to update
     * the sensor data
     */
    virtual void update() = 0;
    /**
     * Initializes the device
     * @param return 1 if success 0 otherwise
     */
    virtual int init() = 0;
    /** Pose of a beam, relative to the parents position */
    CPose* mRelativeBeamPose;
    /** Range data of a beam, range [m], reflectance [0,1] */
    tRangeData* mRangeData;
    /** Number of samples in one scan */
    unsigned int mNumSamples;
    /**
     * Gets the angle of the beam cone, for this like lases the
     * angle is practically 0
     * @return angle [rad]
     */
    float getBeamConeAngle() { return mBeamConeAngle; };
    /**
     * Calculates the response for a point in the sensors local coordinate
     * system from the inverse range sensor model
     * @param x ordinate of point (global cs)
     * @param y ordinate of point (global cs)
     * @param robotPos map center relative position of robot
     * @param heading of robot [rad]
     * @return log odds of obstacle at (x,y)
     */
    float localInverseRangeSensorModel(float x, float y, tPoint2d robotPos,
                                       float heading);
    /**
     * Calculates the response for a point in the sensors local coordinate
     * system from the inverse range sensor model
     * @param x ordinate of point
     * @param y ordinate of point
     * @param robotPose map center relative pose of robot
     * @return log odds of obstacle at (x,y)
     */
    float localInverseRangeSensorModel(float x, float y, CPose robotPose);
    /**
     * Gets the maximum possible range of the sensor
     * @return [m]
     */
    float getMaxRange();
    /**
     * Gets the minimum possible range of the sensor
     * @return [m]
     */
    float getMinRange();
    /**
     * Prints the devices main information
     */
    virtual void print();
    /**
     * Gets the field of view
     * @return [rad]
     */
    float getFov() { return mFov; };

  protected:
    /**
     * Default constructor
     * @param deviceIndex index for this device
     */
    ARangeFinder( int deviceIndex );
    /** Maximum range [m] */
    float mMaxRange;
    /** Minimum range [m] */
    float mMinRange;
    /** Range resolution of a sample [m] */
    float mRangeResolution;
    /** Angle of the beam cone [rad] */
    float mBeamConeAngle;
    /** Log odds of occupied cell */
    float mLOccupied;
    /** Log odds for free cell */
    float mLFree;
    /** Half obstacle thickness [m] */
    float mHalfObstacleThickness;
    /** Log liklihood for prior, basically for no information */
    float mL0;
    /** Field of view [rad] */
    float mFov;
};

#endif
