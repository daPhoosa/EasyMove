# **EasyMove**
Arduino compatible movement engine

* Constant acceleration trajectory planning
* Velocity is modulated at movement junctions based on the sharpness of the corner.  Colinear lines have no speed reduction while a sharp U-turn will force complete deceleration.
* Multiple move queuing for smooth continuous motion
* Supports non-poly line arc moves ( G2 and G3 CNC moves )


## Initialization
* Acceleration should have time units of seconds ( mm/s^2 or in/s^2 )
* maximumSpeed will be used for rapid moves and the upper limit for feed moves
```
EasyMove motion( acceleration, maximumSpeed );
```

## Starting Motion
* Moves must NOT be added to the motion engine before the startMoving() function is run, they will be ignored
```
motion.setPostion( x_start, y_start, z_start );
motion.startMoving();
```

## Adding Moves
* Always check "motion.bufferVacancy()" before adding a motion block to prevent overwriting an active move.  It will return TRUE if there is room.
* All moves positions should be in absolute position
* Any linear distance units can be used as long it is the same for acceleration, velocity and position
* ( mm/s^2, mm/s, mm   or  in/s^2, in/s, in )
* Time units must be seconds for acceleration and velocity
* Arc direction is either 2 or 3 correspinding to standard CNC g-code ( G2 = CW and G3 = CCW )
```
if( motion.bufferVacancy() )  // always check for room in the buffer before adding a block or bad things will happen
{
   motion.addRapidBlock( x, y, z );
   //motion.addLinearBlock( x, y, z, feedRate );
   //motion.addArc_Block( arcDirection, end_x, end_y, feedRate, center_x, center_y );
   //motion.addDwellBlock( delayMS );  // dwell is the lone exception to checking for buffer vacancy, it will behave nicely even if the buffer is full
}
```

## Getting Current Cartesian Position
* Targer position is calculated based on the time at which this function is called.  Early or late calls will always return the correct location for that time.
* It is up to the user to perform any inverse kinematics required to transfrom from cartesian space to motor space in non-cartesian machines.
```
motion.getTargetLocation( x, y, z );   // new position is passed by reference through the three arguments
```

## Pausing Motion
* Motion can be paused by stopping the addition of moves to the movement engine.  Once all queued moves have been executed, the movement will smoothly stop at the end of the last move.
* Motion can be resumed by submitting more moves to the motion engine.
