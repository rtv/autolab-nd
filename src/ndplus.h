/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Ash Charles (jac27@sfu.ca)                                     *
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
 ***************************************************************************/
#ifndef CNDPLUS_H
#define CNDPLUS_H

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "nd.h"
#include "RapiCore"

using namespace Rapi;

/** Type definition for obstacle avoidance */
typedef enum { NONE, STALL, BOTH, RIGHT, LEFT } tObstacle;

/**
 * An extension to ND which does obstacle avoidance using information from
 * both a rangefinder and a bumper.
 * This extension is currently Chatterbox-specific: it expects a particular
 * bumper configuration and initializes ND with parameters tuned for the
 * the Chatterbox IR range finders.
 */
class CNdPlus: public CNd
{
  public:
    /**
     * Default constructor
     * @param bumper 
     * @param rangefinder
     */
    CNdPlus( ABinarySensorArray * bumper, ARangeFinder * ranger,
             std::string name = "Robot");
    /** Default destructor */
    ~CNdPlus();
    /** Get Recommended Velocity */
    CVelocity2d getRecommendedVelocity();
    /**
     * Updates the algorithm
     * @param timestamp current time [s]
     * @param pose current pose of robot (global coordinate system)
     * @param velocity current velocity of robot
     */
    void update( float timestamp, CPose2d pose, CVelocity2d velocity );

  private:
    /** Bumper */
    ABinarySensorArray * mBumper;
    /** Rangefinder */
    ARangeFinder * mRanger;
    /** Time when we last hit an obstacle */
    double mObstacleTime;
    /** Length of time to backup from an obstacle [s] */
    double mBackupTime;
    /** Speed at which to reverse from obstacles */
    double mBackupSpeed;
    /** Length of time to turn away from an obstacle [s] */
    double mTurnTime;
    /** Amount of yaw to turn away from an obstacle [rad] */
    double mTurnThreshold;
    /** Rate at which to turn away from an obstacle [rad/s] */
    double mTurnRate;
    /** Recommended Forward Velocity */
    double mVRec;
    /** Recommended Yaw Rate */
    double mWRec;
    /** Cause of current obstacle **/
    tObstacle mObstacle;
    /** A random number to break deadlocks */
    double mRandom;
    /** Rear range */
    double mRearRange;
    /** Rear threshold: stop backing up if something is closer than this */
    double mRearThreshold;
};


#endif // CNDPLUS_H
