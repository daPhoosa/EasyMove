# **EasyMove**
Arduino compatible movement engine

* Constant acceleration trajectory planning
* Multiple move queuing


## Initialization
```
EasyMove motion( acceleration, maximumSpeed );
```

## Starting Motion
```
motion.setPostion( x_start, y_start, z_start );
motion.startMoving();
```

## Adding Moves
```
if( motion.bufferVacancy() )  // always check for room in the buffer before adding a block or bad things will happen
{
   motion.addRapidBlock( x, y, z );
   //motion.addLinearBlock( x, y, z, feedRate );
   //motion.addArc_Block( arcDirection, x, y, feedRate, centerX, centerY );
   //motion.addDwellBlock( delayMS );
}
```

## Getting Current Position
```
motion.getTargetLocation( x, y, z );   // new position is passed by reference through the three arguments
```

