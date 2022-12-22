#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motor1               motor         1               
// Motor2               motor         2               
// Motor8               motor         8               
// Motor9               motor         9               
// Rotation3            rotation      3               
// Rotation10           rotation      10              
// Inertial4            inertial      4               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your robot manually with the sidebar configurer. */
/*---------------------------------------------------------------------------*/

//Add your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).

Drive chassis(

//Add your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).

//Left Motors:
motor_group(Motor1, Motor2),

//Right Motors:
motor_group(Motor8, Motor9),

//Inertial
Inertial4,

//Wheel Diameter (4" is really 4.125")
4.125,

//External ratio, must be in decimal, output speed/input speed
1.3333,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
360

);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int current_auton_selection = 0;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  while(1){
    Brain.Screen.clearScreen();
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(50, 50, "No Auto");
        break;
      case 1:
        Brain.Screen.printAt(50, 50, "Skills");
        break;
      case 2:
        Brain.Screen.printAt(50, 50, "Left");
        break;
      case 3:
        Brain.Screen.printAt(50, 50, "6 Disc Left");
        break;
      case 4:
        Brain.Screen.printAt(50, 50, "Right");
        break;
      case 5:
        Brain.Screen.printAt(50, 50, "AWP");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 6){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

void autonomous(void) {
  switch(current_auton_selection){  
    case 0:
      drive_test(); //This is the default auton, if you don't select from the brain.
      break;
    case 1:
      drive_test();
      break;
    case 2:
      turn_test();
      break;
    case 3:
      swing_test();
      break;
    case 4:
      full_test();
      break;
    case 5:
      full_test();
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}