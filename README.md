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
* Always check `motion.bufferVacancy()` before adding a motion block to prevent overwriting an active move.  It will return TRUE if there is room.
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

## Pausing/Stopping Motion
* Motion can be paused by stopping the addition of moves to the movement engine.  Once all queued moves have been executed, the movement will smoothly stop at the end of the last move.
* Motion can be resumed by submitting more moves to the motion engine.
* Motion can be forcefully stopped using the `motion.abortMotion()` function.  This will delete all moves and may produce unpredictable results if executed while moving.
* It is possbile to check if all moves have been completed using the `motion.blockQueueComplete()` function.  This will return true if there are no moves queud up or if all queud moves have been completed.

## List of public functions
```
bool bufferVacancy();
```
>* This will return TRUE if there is room in the buffer for another move
##

```
void addRapid_Block(  float _x, float _y, float _z );
```
>* Add a linear move at max velocity to this point
##

```
void addLinear_Block( float _x, float _y, float _z, float _feed );
```
>* Add a linear move at the specified velocity  to this point
##

```
void addArc_Block( int type, float _x, float _y, float _feed, float centerX, float centerY );
```
>* Add an arc move in the XY plane at the current z position.
>* Pass arc direction, end point, feed rate and center point
>* Arc direction: 2 = CW and 3 = CCW ( follows standard CNC G2 and G3 )
>* If the end point matches the start point, a full circle will be produced
>* If the start-to-center radius and end-to-center radius are excessively different, the motion engine will hang (todo: handle this more gracefully )
>* This is slightly less efficient than a poly line arc when executing _getTargetLocation( ... )_ during motion control
##

```
void addDwell_Block( int delayMS );
```
>* This adds a motion block of zero length to the queue and then appends a dwell to the end of it
>* If there is no room in the block buffer, the dwell will be appended to the end of the last move.  This makes is safe to add a dwell at any time, even withour checking for space in the buffer.
##

```
void setPosition( float t_x, float t_y, float t_z );
```
>* This sets the current position (start position) of the machine before `startMoving()` is executed
>* Don't call this while moving or bad things will happen
##

```
void setLookAheadTime(int timeMS );
```
>* Use this to set the minimum amount of movement time in the buffer.
>* Default is 250ms
>* This can be used to save processing time by only queuing up enough blocks to insure the motion is not being throttled
##

```
void setJunctionVelRad( float t_r );
```
>* This is used to adjust the speed at which the motion engine will cross line junctions.
>* Default is 0.050mm
>* If the motion engine is too cautious at corners, increase.  If it is too fast, decrease.
##

```
void startMoving();
```
>* Execute this to start motion
>* This must be run before adding motion blocks to the buffer
##

```
void abortMotion();
```
>* Stops motion abruptly, requires `startMoving()` to continue again
##

```
float setMotionRateOverride(  float scale );
```
>* Scale feed rate up or down, 1.0 is no speed change
>* Only effects moves added _after_ a change
>* Will not exceed max feed rate set on object initialization
##

```
void getTargetLocation( float & x, float & y, float & z );
```
>* Used to get the current cartesin position during motion control
##

```
float getSpeed();
```
>* Returns the speed of the tool in the direction of motion
##

```
int  getBlockCount();
```
>* Returns the number of motion block currently in the queue
>* This should _not_ be used to determine if all moves are complete ( see: `blockQueueComplete()` )
##

```
bool blockQueueComplete();
```
>* Returns TRUE once all moves in the buffer are complete
##


