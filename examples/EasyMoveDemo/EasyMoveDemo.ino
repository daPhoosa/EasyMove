/*
 *  EasyMove Demo program
 *  
 *  This program sends a series of move commands to the movement engine while 
 *  it simultaneously sends the current target location to the serial port
 *  in real time taking into account proper acceleration and deceleration
 *  so that corners and dwells are negotiated smoothly.
 *  
 */

#include <EasyMove.h> // installed to arduino libraries directory

const float acceleration = 500.0f;    // mm/s^2
const float maximumVelocity = 120.0;  // mm/s

float start_x = 15.0;   // mm
float start_y = -5.0;  // mm
float start_z = 10.0;   // mm

float feedRate = 60.0; // mm/s

EasyMove motion( acceleration, maximumVelocity ); // create motion control object

void setup() 
{
  Serial.begin(250000);

  motion.setPosition( start_x, start_y, start_z ); // Start location must be set before beginning motion
  motion.startMoving();
}

void loop() 
{
  do_motion_control();

  do_other_tasks();

  dwell_until_next_loop();
}

void do_motion_control()
{
  float x, y, z;

  motion.getTargetLocation( x, y, z ); // get the current location from the motion controller

  your_motor_controller( x, y, z ); // send the new position to your motor controller
}

void do_other_tasks()
{
  if( motion.bufferVacancy() ) // always check for room in the buffer before adding blocks
  {
    add_next_move();
  }
  else if( motion.blockQueueComplete() )  // us this function to check if there are any moves still being executed
  {
    Serial.println("Program Execution Complete");
  }
}

void your_motor_controller( float x, float y, float z )
{
  // control you motors to reduce the delta between the target location and their current locations

  if( !motion.blockQueueComplete() ) // only display position data while executing program
  {
    // since we have no motors, sending the data to the serial port allows for visualization
    Serial.print( x, 3); Serial.print("\t"); Serial.print( y, 3); Serial.print("\t"); Serial.print( z, 3);
    Serial.println("");
  }
}

void add_next_move()
{
  static int index = 0;

  switch( index )
  {
    case 0:
      motion.addRapid_Block( 0.0, 0.0, 0.0 );
      motion.addDwell_Block( 10 ); // 10ms dwell -- addDwellBlock can excuted without checking for buffer room
      break;

    case 1:
      motion.addLinear_Block( 10.0, 0.0, 0.0, feedRate );
      break;

    case 2:
      motion.addLinear_Block( 10.0, 20.0, 0.0, feedRate );
      break;

    case 3:
      motion.addLinear_Block( -10.0, 20.0, 0.0, feedRate );
      break;

    case 4:
      motion.addLinear_Block( -10.0, 0.0, 0.0, feedRate );
      break;

    case 5:
      motion.addLinear_Block( 0.0, 0.0, 0.0, feedRate );
      break;

    case 6:
      motion.addLinear_Block( 9.0, 9.0, 0.0, feedRate );
      break;

    case 7:
      motion.addLinear_Block( 0.0, 8.6, 0.0, feedRate );
      break;

    case 8:
      motion.addLinear_Block( -9.0, 9.0, 0.0, feedRate );
      break;

    case 9:
      motion.addLinear_Block( 0.0, 0.0, 0.0, feedRate );
      motion.addDwell_Block( 10 ); // 10ms dwell -- addDwellBlock can excuted without checking for buffer room
      break;

    case 10:
      motion.addRapid_Block( -start_x, start_y, start_z );
      break;

    default:
      index = 100; // prevent int overflow and blocks being re-submitted
      break;
  }
  index++;
}

void dwell_until_next_loop()
{
  // This is a TERRIBLE way to control loop execution time
  // Don't do this in a real application.  It is only done here for simplicities sake.

  static uint32_t nextExecuteTime;

  while( micros() < nextExecuteTime );

  nextExecuteTime = micros() + 5000; // run at ~200Hz
}

