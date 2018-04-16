/*
   EasyMove
   Phillip Schmidt
   v0.1

         This program is free software: you can redistribute it and/or modify
         it under the terms of the GNU General Public License as published by
         the Free Software Foundation, either version 3 of the License, or
         (at your option) any later version.

         This program is distributed in the hope that it will be useful,
         but WITHOUT ANY WARRANTY; without even the implied warranty of
         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
         GNU General Public License for more details.

         You should have received a copy of the GNU General Public License
         along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include "EasyMove.h"
#include "motion.h"
#include "blockBuffer.h"
#include "control.h"


EasyMove::EasyMove( float accel, float maxVelocity )
{
   // precompute to save time when adding blocks
   maxAccel         = accel;
   accelInverse     = 1.0f / accel;
   accelInverseHalf = 0.5f * accelInverse;
   accelDouble      = 2.0f * accel;

   maxVel = maxVelocity;

   motionFeedOverride  = 1.0f;

   setJunctionVelRad( 0.05f );

   lookAheadTimeMin = 250000;  // [us]

   motionStopped = true;
}

