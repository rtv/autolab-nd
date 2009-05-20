/***************************************************************************
 * Project: RAPI                                                           *
 * Author:  Jens Wawerla (jwawerla@sfu.ca)                                 *
 * $Id: $
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
 **************************************************************************/
#ifndef WANDERCTRL_H
#define WANDERCTRL_H

#include "RapiLooseStage"

using namespace Rapi;
/**
 * Example robot controller, based on the wander example from Stage
 * @author Jens Wawerla
 */
class CWanderCtrl : public ARobotCtrl
{
  public:
    /**
     * Default constructor
     * @param robot to control
     */
    CWanderCtrl(ARobot* robot);
    /** Default destructor */
    ~CWanderCtrl();

  protected:
    /**
     * Update controller for the current time step
     * @param dt time since last upate [s]
     */
    void updateData(float dt);

    /** Stage position model */
    CLooseStageDrivetrain2dof* mDrivetrain;
    /** Stage laser model */
    CLooseStageLaser* mLaser;
    /** Stage power pack */
    CLooseStagePowerPack* mPowerPack;
    /** Text display */
    CLooseStageTextDisplay* mTextDisplay;
    /** Fiducial Finder */
    CLooseStageFiducialFinder* mFiducial;
    /** Count avoid time steps */
    int mAvoidcount;
    /** Just for test purpose */
    bool mFgAvoid;
    /** Just for test purpose */
    int mCounter;
};

#endif
