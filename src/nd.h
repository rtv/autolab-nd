/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: nd.h,v 1.5 2009-03-29 00:54:27 jwawerla Exp $
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
 * $Log: nd.h,v $
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
 *
 ***************************************************************************/
#ifndef CND_H
#define CND_H

#include "nd_alg.h"
#include "pose2d.h"
#include "velocity2d.h"
#include "utilities.h"
#include "error.h"
#include "rangefinder.h"

const int MAX_ND_SENSORS = 3;
using namespace Rapi;

/**
 * Rapi interface to Nearest Distance obstacle avoidance. This is a
 * C++ class that wraps the ND algorithm implemented by J. Minguez
 * and is based on the following publications:
 *
 * J. Minguez, L. Montano Nearness Diagram
 * Navigation (ND): Collision Avoidance in Troublesome Scenarios. In
 * IEEE Transactions on Robotics and Automation, pp 154, 2004.
 *
 * J. Minguez, J. Osuna, L. Montano.
 * A Divide and Conquer Strategy based on Situations to Achieve Reactive
 * Collision Avoidance in Troublesome Scenarios. In IEEE International
 * Conference on Robotics and Automation (ICRA 2004), 2004. New Orleans, USA.
 *
 * Minguez's source code contains no license information, but has been
 * distributed with Player (http://playerstage.org) for several years
 * under LGPLv2. The nd_alg.h/cpp files are taken from the Player
 * distribution.
 *
 * @author Jens Wawerla <jwawerla@sfu.ca>
 * @verion 0.1 - 01/2008
 */
class CNd
{
  public:
    /** Default constructor */
    CNd ( const char* robotname = NULL );
    /** Default destructor */
    ~CNd();
    /**
     * Adds a rangefinder to the sensor list
     * @param sensor to be added
     */
    void addRangeFinder ( ARangeFinder* sensor );
    /**
     * Gets the recommended velocity
     * @return [m/s] [rad/s]
     */
    CVelocity2d getRecommendedVelocity();
    /**
     * Gets the recommended translational velocity
     * @return [m/s]
     */
    float getRecommendedTranslationalVelocity() { return mVCmd; };
    /**
     * Gets the recommended rotational velocity
     * @return [rad/s]
     */
    float getRecommendedRotationalVelocity() { return mWCmd; };
    /**
     * Updates the algorithm
     * @param pose current pose of robot (local coordinate system)
     */
    void update ( CPose2d pose );
    /**
     * Sets the goal for the algorithm in robot local coordinates
     * @param goal to get to in robot local coordinates
     */
    void setGoal ( CPose2d goal );
    /**
     * Gets the current goal
     * @return goal
     */
    CPose2d getGoal() { return mGoal; };
    /**
     * Checks if we have an active goal, an active goal is a goal that
     * is set but has not been reached yet
     * @return true if we have an active goal, false otherwise
     */
    bool hasActiveGoal();
    /**
     * Checks if the robot is stalmLed
     * @return true if stalled, false otherwise
     */
    bool isStalled();
    /**
     * Checks if Nd suggest to turn in place
     * @return true if turn in place recommended, false other wise
     */
    bool isTurningInPlace() { return mTurningInPlace; };
    /**
     * Have we reached the current goal ?
     * @return true if goal was reached, false otherwise
     */
    bool atGoal();
    /**
     * Set the angular difference between the current robot heading and
     * the goal heading that is considered as objective met
     * @param angle [rad]
     */
    void setEpsilonAngle ( float angle );
    /**
     * Set the distance to a waypoint that is considered objective met
     * @param dist [m]
     */
    void setEpsilonDistance ( float dist );
    /**
     * Resets Nd's state variables
     */
    void reset();

  protected:
    /** Processes information from all registered sensors */
    void processSensors();
    /**
     * Computes the signed minimum difference between the two angles
     * @param a angle [rad]
     * @param b angle [rad]
     * @return [rad]
     */
    float angleDiff ( float a, float b );
    /**
     * Sets the driving direction
     * @param dir 1 forward, 0 backwards
     */
    void setDirection ( int dir );
    /**
     * Threshold a given velocity to {[-vMin, -vMax], 0, [vMin, vMax]}
     * @param vMin minimal velocity
     * @param vMax maximal velocity
     * @return thresholded velocity
     */
    float threshold ( float v, float vMin, float vMax );

  private:
    /** Name of robot */
    char mRobotName[20];
    /** Pose of robot */
    CPose2d mRobotPose;
    /** Pose of robot from last time step */
    CPose2d mLastRobotPose;
    /** Goal to drive to */
    CPose2d mGoal;
    /** Distance epsilon, basically a threshold when things are close enough [m] */
    float mDistEps;
    /** Angle epsilon, basically a threshold when things are close enough [rad] */
    float mAngleEps;
    /** List of rangefinders to be used */
    ARangeFinder* mSensorList[MAX_ND_SENSORS];
    /** Number of registered sensors */
    int mNumSensors;
    /** Parameter set for ND algorithm */
    TParametersND mNDparam;
    /** Maximal translational velocity [m/s] */
    float mVMax;
    /** Minimal translational velocity [m/s] */
    float mVMin;
    /** Maximal rotational velocity  [rad/s] */
    float mWMax;
    /** Minimal rotational velocity  [rad/s] */
    float mWMin;
    /** Safety distance to obstacles [m] */
    float mSafetyDist;
    /** Recommended translational velocity [m/s] */
    float mVCmd;
    /** Recommended rotational velocity [rad/s] */
    float mWCmd;
    /** Maximal translational acceleration [m/s^2] */
    float mVDotMax;
    /** Maximal rotational acceleration [rad/s^2] */
    float mWDotMax;
    /**
     * Time constant for rotational vel. low pass filter for turn in
     * place situations [s]
     */
    float mTauWTurnInPlace;
    /** Time constant for rotational vel. low pass filter for normal turning[s] */
    float mTauW;
    /** Distance at which obstacle avoidance begins [m] */
    float mAvoidDist;
    float mRotateMinError;
    float mRotateStartTime;
    /**
     * How long the robot is allowed to rotate in place without making any
     * progress toward the goal orientation before giving up [s]
     */
    float mRotateStuckTime;
    float mTranslateStartTime;
    float mTranslateMinError;
    /**
     * How long the robot is allowed to translate without making sufficient
     * progress toward the goal position before giving up [s]
     */
    float mTranslateStuckTime;
    /**
     * How far the robot must translate during translate stuck time in
     * order to not give up [m]
     */
    float mTranslateStuckDist;
    /**
     * How far the robot must rotate during translate stuck time in order
     * to not give up [rad]
     */
    float mTranslateStuckAngle;
    /** Current driving direction 1 forward, 0 backwards */
    int mCurrentDir;
    /** Are we stalmLed */
    bool mFgStall;
    /**
     * Should local navigation be paused if the stall flag is set on the
     * input:::position2d device? This option is useful if the robot's pose
     * is being read from a SLAM system that sets the stall flag when it
     * is performing intensive computation and can no longer guarantee the
     * validity of pose estimates
     */
    bool mFgWaitOnStall;
    bool mTurningInPlace;
    bool mFgWaiting;
    /** Do we have an active goal ? */
    bool mFgActiveGoal;
    /** Have we reached the goal ? */
    bool mFgAtGoal;
    /** List of obstacles */
    TInfoEntorno mObstacles;
    int mDir;
};

#endif
