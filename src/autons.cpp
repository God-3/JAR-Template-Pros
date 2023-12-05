#include "main.h"

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 4000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

// Autonomous Procedures
void far(){
    /*chassis.drive_distance(43, 7, 1, 10, 4000, 1.5, 1, 10, 0);
    pros::delay(10);
    chassis.turn_to_angle(70, 8, 5, 10, 4000, 1, .03, 1, 0);
    intake.move(-120);
    pros::delay(150);
    chassis.drive_distance(4, 7, 0.5, 10, 750, 1.5, 1, 10, 0);
    intake.move(0);
    pros::delay(60);
    //first triball scored
    chassis.drive_distance(1, -7, 0.5, 10, 130, 1.5, 1, 10, 0);
    pros::delay(10);
    chassis.left_swing_to_angle(12, 7, 12, 20, 4000, 1, .03, 1, 0);
    intake.move(120);
    pros::delay(100);
    chassis.drive_distance(17, 7, 0.5, 10, 4000, 1.5, 1, 10, 0);
    pros::delay(150);
    intake.move(0);
    //pickup second triball
    chassis.turn_to_angle(73, 8, 9, 20, 4000, 1, .03, 1, 0);
    intake.move(-120);
    pros::delay(150);
    chassis.drive_distance(19, 7, 1, 10, 1000, 1.5, 1, 10, 0);
    intake.move(0);
    pros::delay(60);
    //second triball scored
    chassis.drive_distance(5, -7, 0.5, 10, 285, 1.5, 1, 10, 0);
    chassis.turn_to_angle(300, 7, 10, 20, 4000, 1, .03, 1, 0);
    intake.move(120);
    pros::delay(100);
    chassis.drive_distance(15, 7, 1, 10, 1400, 1.5, 1, 10, 0);
    pros::delay(220);
    intake.move(0);
    // 3rd triball picked up
    chassis.drive_distance(22, -7, 1, 10, 500, 1.5, 1, 10, 0);
    pros::delay(50);
    chassis.turn_to_angle(90, 6, 10, 20, 4000, 1, .03, 1, 0);
    intake.move(-120);
    pros::delay(225);
    chassis.drive_distance(5, 7, 1, 10, 800, 1.5, 1, 10, 0);
    intake.move(0);
    pros::delay(20);
    chassis.drive_distance(4, -7, 1, 10, 500, 1.5, 1, 10, 0);
    pros::delay(50);
    chassis.turn_to_angle(250, 7, 10, 20, 4000, 1, .03, 1, 0);
    pros::delay(50);
    pneumatics.wings_v(1);
    chassis.drive_distance(8, -7, 1, 10, 800, 1.5, 1, 10, 0);*/
    //third triball scored
    //def not copied 5 triball
    chassis.drive_distance(1, 5, 1, 10, 10, 1.5, 1, 10, 0);
    intake.move(120);
    pros::delay(350);
    intake.move(20);
    chassis.drive_distance(29, -8, 1, 10, 865, 1.5, 1, 10, 0);
    intake.move(0);
    chassis.left_swing_to_angle(315, 6, 12, 20, 4000, 1, .03, 1, 0);
    pneumatics.wings_v(1);
    pros::delay(350);
    chassis.drive_distance(19, -11, 1, 10, 400, 1.5, 1, 10, 0);
    chassis.left_swing_to_angle(280, 4, 12, 20, 4000, 1, .03, 1, 0);
    pneumatics.wings_v(0);
    pros::delay(400);
    //takes out match-load triball
    chassis.drive_distance(18, -8, 1, 10, 400, 1.5, 1, 10, 0);
    //rams match-load and pre-load triball
    chassis.drive_distance(5, 11, 1, 10, 200, 1.5, 1, 10, 0);
    chassis.turn_to_angle(95, 4, 15, 10, 4000, 1, .03, 1, 0);
    intake.move(-120);
    pros::delay(125);
    //outakes triball in intake
    chassis.drive_distance(5, 11, 1, 10, 200, 1.5, 1, 10, 0);
    chassis.drive_distance(5, -11, 1, 10, 200, 1.5, 1, 10, 0);
    chassis.turn_to_angle(290, 4, 15, 10, 4000, 1, .03, 1, 0);
    chassis.drive_distance(10, -11, 1, 10, 360, 1.5, 1, 10, 0);
    intake.move(0);
    //done shoving 3 triballs
    chassis.left_swing_to_angle(0, 8, 12, 20, 4000, 1, .03, 1, 0);
    chassis.drive_distance(9, 11, 1, 10, 225, 1.5, 1, 10, 0);
    chassis.turn_to_angle(35, 9, 10, 20, 4000, 1, .03, 1, 0);
    chassis.drive_distance(22, 11, 1, 10, 700, 1.5, 1, 10, 0);
    intake.move(120);
    chassis.turn_to_angle(25, 6, 6, 20, 1000, .4, .03, 3, 0);
    chassis.drive_distance(3, 11, 1, 10, 325, 1.5, 1, 10, 0);
    pros::delay(275);
    intake.move(0);
    pneumatics.wings_v(1);
    pros::delay(200);
    chassis.drive_distance(40, -11, 1, 10, 1500, 1.5, 1, 10, 0);
    pneumatics.wings_v(0);
    pros::delay(100);
    chassis.drive_distance(5, 11, 1, 10, 300, 1.5, 1, 10, 0);
    chassis.turn_to_angle(180, 6, 10, 20, 4000, .4, .03, 3, 0);
    intake.move(-120);
    pros::delay(100);
    chassis.drive_distance(7, 11, 1, 10, 500, 1.5, 1, 10, 0);
    intake.move(0);
    chassis.drive_distance(4, -11, 1, 10, 500, 1.5, 1, 10, 0);
    chassis.turn_to_angle(40, 6, 10, 20, 4000, .4, .03, 3, 0);
    pneumatics.wings_v(1);
    chassis.drive_distance(8, -11, 1, 10, 800, 1.5, 1, 10, 0);
}

void close_qual(){
  //AWP
	chassis.drive_distance(7, -5, 1, 10, 4000, 1, 1, 1, 0);
	pneumatics.wings_v(1);
	pros::delay(250);
  chassis.turn_to_angle(270, 7, 10, 20, 4000, 1, 1, 1, 0);
  pneumatics.wings_v(0);
  pros::delay(250);
  chassis.drive_distance(28, 10, 1, 80, 920, 1, 1, 1, 0);
	pros::delay(100);
	chassis.turn_to_angle(121, 4, 10, 20, 1000, 1, 1, 1, 0);
	pros::delay(100);
	intake.move(-120);
	pros::delay(250);
	chassis.drive_distance(18, 8, 2, 20, 2000, 1, 1, 1, 0);
	intake.move(0);
  chassis.turn_to_angle(135, 4, 10, 20, 1000, 1, 1, 1, 0);
  pros::delay(250);
	chassis.drive_distance(20, -5, 1, 10, 2000, 1.5, 1, 10, 0);
	chassis.drive_distance(2, 8, 1, 10, 500, 1.5, 1, 10, 0);
	pros::delay(250);
	chassis.turn_to_angle(220, 4, 10, 20, 4000, 1.5, 1, 10, 0);
	pros::delay(100);
  pneumatics.wings_v(1);
	chassis.drive_distance(10, -8, 1, 10, 3000, 1.5, 1, 10, 0);
	pros::delay(250);
	chassis.turn_to_angle(245, 4, 10, 20, 3000, 1.5, 1, 10, 0);
}

void close_elim(){
  chassis.drive_distance(43, 7, 1, 10, 4000, 1.5, .03, 10, 0);
  pros::delay(10);
  chassis.turn_to_angle(290, 8, 5, 10, 4000, .4, .03, 3, 0);
  intake.move(-120);
  pros::delay(150);
  chassis.drive_distance(6, 7, 0.5, 10, 900, 1.5, .03, 10, 0);
  intake.move(0);
  pros::delay(60);
  chassis.turn_to_angle(280, 4, 10, 20, 1000, .4, .03, 3, 0);
  pros::delay(50);
  pneumatics.wings_v(1);
  pros::delay(250);
	chassis.drive_distance(20, -5, 1, 10, 2000, 1.5, 1, 10, 0);
  pneumatics.wings_v(0);
  pros::delay(200);
  chassis.turn_to_angle(230, 4, 10, 20, 1000, .4, .03, 3, 0);
  pros::delay(50);
  chassis.drive_distance(45, 6, 1, 10, 2000, 1.5, 1, 10, 0);
  chassis.turn_to_angle(260, 4, 10, 20, 1000, .4, .03, 3, 0);
  chassis.drive_distance(10, 6, 1, 10, 1000, 1.5, 1, 10, 0);
  //Scores triball and goes to match-loading bkar
  /*chassis.drive_distance(22, -8, 1.5, 50, 1500, 1.5, 0, 10, 0); 
  chassis.turn_to_angle(30, 9, 3, 50, 1500, .4, .03, 3, 15); 
  chassis.drive_distance(3, -10, 1.5, 50, 300, 1.5, 0, 10, 0); 
  chassis.drive_distance(5, 10, 1.5, 50, 300, 1.5, 0, 10, 0); 
  chassis.turn_to_angle(98, 9, 3, 50, 1500, .4, .03, 3, 15); 
  chassis.drive_distance(2, 3, 1.5, 50, 500, 1.5, 0, 10, 0); */
}

void skills(){
    //Auton skills
    chassis.drive_distance(22, -8, 1.5, 50, 1500, 1.5, 0, 10, 0); 
    chassis.turn_to_angle(30, 9, 3, 50, 1500, .4, .03, 3, 15); 
    chassis.drive_distance(3, -10, 1.5, 50, 300, 1.5, 0, 10, 0); 
    chassis.drive_distance(5, 10, 1.5, 50, 300, 1.5, 0, 10, 0); 
    chassis.turn_to_angle(98, 9, 3, 50, 1500, .4, .03, 3, 15); 
    chassis.drive_distance(2, 3, 1.5, 50, 500, 1.5, 0, 10, 0); 
    slapper.move(120); 
    pros::delay(3000); 
    slapper.move(0);
    int count = 0;
    while (true){
    if (chassis.get_rotation() > 23000 and chassis.get_rotation() < 32000){
      slapper.move(0);
      break;
    }
    if (count > 1500){
      break;
    }
    pros::delay(5); 
    count += 5;
    }
    //done match-loading
    chassis.turn_to_angle(170, 8, 5, 20, 1500, .4, .03, 3, 15); 
    chassis.drive_distance(8, -8, 1.5, 50, 600, 1.5, 0, 10, 0); 
    chassis.turn_to_angle(124, 8, 3, 50, 1500, .4, .03, 3, 15); 
    chassis.drive_distance(76, -8, 1.5, 50, 1640, 1.5, 0, 10, 0);
    //pneumatics.wings_v(1);
    //pros::delay(250);
    chassis.turn_to_angle(76, 8, 3, 50, 1520, .4, .03, 3, 15);
    chassis.drive_distance(27, -8, 1.5, 20, 1500, 1.5, 0, 10, 0);
    chassis.turn_to_angle(30, 3, 3, 50, 1500, .4, .03, 3, 15);
    //pneumatics.wings_v(0);
    //pros::delay(250);
    //reached the other side of the field, 
    chassis.drive_distance(7, -8, 1, 10, 850, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(4, 5, 1, 10, 300, 1.5, 0, 10, 0);
    chassis.turn_to_angle(34, 8, 3, 50, 1500, .4, .03, 3, 15);
    pros::delay(50);
    chassis.drive_distance(10, -8, 1, 10, 700, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(4, 5, 1, 10, 300, 1.5, 0, 10, 0);
    chassis.turn_to_angle(35, 8, 3, 50, 1500, .4, .03, 3, 15);
    pros::delay(50);
    chassis.drive_distance(10, -8, 1, 10, 700, 1.5, 0, 10, 0);
    chassis.turn_to_angle(30, 8, 3, 50, 1500, .4, .03, 3, 15);
    pros::delay(50);
    //pushed #1 complete
    chassis.drive_distance(3, 7, 1, 10, 300, 1.5, 0, 10, 0);
    chassis.turn_to_angle(320, 8, 3, 50, 1500, .4, .03, 3, 15);
    chassis.drive_distance(39, -5, 2, 40, 2000, 1.5, 0, 10, 0);
    pros::delay(150);
    pneumatics.wings_v(1);
    pros::delay(250);
    chassis.turn_to_angle(80, 5, 3, 50, 1500, .4, .03, 3, 15);
    chassis.drive_distance(25, -8, 1.5, 50, 1050, 1.5, 0, 10, 0);
    chassis.turn_to_angle(90, 8, 3, 50, 1500, .4, .03, 3, 15);
    pros::delay(50);
    chassis.drive_distance(8, 8, 1.5, 50, 400, 1.5, 0, 10, 0);
    chassis.turn_to_angle(90, 8, 3, 50, 1500, .4, .03, 3, 15);
    pros::delay(50);
    chassis.drive_distance(21, -8, 1.5, 50, 740, 1.5, 0, 10, 0);
    chassis.turn_to_angle(90, 8, 3, 50, 1500, .4, .03, 3, 15);
    pneumatics.wings_v(0);
    pros::delay(350);
    chassis.drive_distance(10, 8, 1.5, 50, 540, 1.5, 0, 10, 0);
    //pushed #2 complete
    chassis.turn_to_angle(30, 8, 3, 50, 1500, .4, .03, 3, 15);
    chassis.drive_distance(11  , -8, 1.5, 50, 400, 1.5, 0, 10, 0);
    pneumatics.wings_v(1);
    pros::delay(200);
    chassis.right_swing_to_angle(300, 5, 5, 10, 1500, .4, .03, 3, 15);
    chassis.drive_distance(16, -8, 1.5, 50, 640, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(5, 8, 1.5, 50, 440, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(21, -8, 1.5, 50, 740, 1.5, 0, 10, 0);
    pros::delay(150);
    pneumatics.wings_v(0);
    pros::delay(350);
    chassis.drive_distance(4, 8, 1.5, 50, 540, 1.5, 0, 10, 0);
    pros::delay(150);
    //pushed #3 complete
    chassis.turn_to_angle(2, 5, 3, 50, 1500, .4, .03, 3, 15);
    chassis.drive_distance(26, -8, 1.5, 50, 750, 1.5, 0, 10, 0);
    pneumatics.wings_v(1);
    pros::delay(200);
    chassis.turn_to_angle(155, 5, 3, 50, 1500, .4, .03, 3, 15);
    chassis.drive_distance(18, -8, 1.5, 50, 540, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(5, 8, 1.5, 50, 440, 1.5, 0, 10, 0);
    pros::delay(150);
    chassis.drive_distance(18, -8, 1.5, 50, 740, 1.5, 0, 10, 0);
    //pushed #4 complete*
  }

void SWP(){
  //Solo Win Point
  chassis.drive_distance(7, -4, 1, 10, 4000, 1, 1, 1, 0);
	pneumatics.wings_v(1);
	pros::delay(250);
  chassis.turn_to_angle(270, 4, 10, 20, 4000, 0.4, 0.03, 1, 0);
  pneumatics.wings_v(0);
  pros::delay(250);
  chassis.drive_distance(30, 4, 1, 80, 2500, 1, 1, 1, 0);
	pros::delay(100);
	chassis.turn_to_angle(130, 4, 10, 20, 1000, 1, 1, 1, 0);
	pros::delay(100);
	chassis.drive_distance(24, 3, 2, 20, 2500, 1, 1, 1, 0);
	intake.move(0);
  slapper.move(120);
  pros::delay(1500);
  slapper.move(0);
  pros::delay(250);
	chassis.drive_distance(20, -5, 1, 10, 2000, 1.5, 1, 10, 0);
	chassis.drive_distance(2, 8, 1, 10, 500, 1.5, 1, 10, 0);
	pros::delay(50);
	chassis.turn_to_angle(220, 4, 10, 20, 4000, 1.5, 1, 10, 0);
	pros::delay(50);
  pneumatics.wings_v(1);
	chassis.drive_distance(10, -8, 1, 10, 1800, 1.5, 1, 10, 0);
	chassis.turn_to_angle(245, 4, 10, 20, 3000, 1.5, 1, 10, 0);
}

void test(){
  slapper.move(120);
  pros::delay(2500);
  int count = 0;
  while (true){
    if (chassis.get_rotation() > 23000 and chassis.get_rotation() < 32000){
      slapper.move(0);
      break;
    }
    if (count > 1500){
      break;
    }
    pros::delay(5); 
    count += 5;
  }

}
