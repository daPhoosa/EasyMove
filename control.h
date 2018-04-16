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


void EasyMove::setPosition( float t_x, float t_y, float t_z )
{
   motionStopped = true;

   X_end = t_x;
   Y_end = t_y; 
   Z_end = t_z;
}


float EasyMove::setMotionRateOverride(  float scale )
{
   motionFeedOverride = constrain( scale, 0.1f, 2.0f );
   return motionFeedOverride;
}


float EasyMove::getSpeed()
{
   return velocityNow;
}


void EasyMove::setLookAheadTime(int timeMS )
{
   lookAheadTimeMin = max( 5, timeMS ) * 1000UL; // time in microseconds
}


int EasyMove::getBlockCount()
{
   return blockCount;
}


void EasyMove::setJunctionVelRad( float t_r )
{
   junctionRadius     = max( t_r, 0.001f );
   junctionRadiusSq   = t_r * t_r;
}
