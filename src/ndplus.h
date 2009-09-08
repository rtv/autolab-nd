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

#include <string.h>
#include "nd.h"
#include "pose2d.h"
#include "velocity2d.h"
#include "binarysensorarray.h"
#include "rangefinder.h"
#include "rectangle.h"
#include <vector>
#include "printerror.h"


using namespace Rapi;

/** Type definition for obstacle avoidance */
typedef enum { NONE, STALL, RIGHT, LEFT } tObstacle;

/**
 * A Chatterbox-specific extension to ND which does obstacle avoidance.
 * Firstly, this supplements ND with information from the bumper.  Secondly,
 * it initializes ND with reasonable parameters for the chatterbox IR
 * rangefinder.
 */
class CNdPlus: public CNd
{
  public:
    /**
     * Default constructor
     * @param chatterbox bumper
     * @param chatterbox ir rangefinder
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
    /** Chatterbox Bumper */
    ABinarySensorArray * mBumper;
    /** Chatterbox Range Finder */
    ARangeFinder * mRanger;
    /** Time when we last hit an obstacle */
    double mObstacleTime;
    /** Length of time to avoid obstacles [s] */
    double mEvadeTime;
    /** Speed at which to reverse from obstaces */
    double mEvadeSpeed;
    /** Recommended Forward Velocity */
    double mVRec;
    /** Recommended Yaw Rate */
    double mWRec;
    /** Cause of current obstacle **/
    tObstacle mObstacle;
};


#endif // CNDPLUS_H
