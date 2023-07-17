#pragma once // Redundant but makes the linter shut up
#ifndef _JAR_AUTONS_H_
#define _JAR_AUTONS_H_
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();

#endif // _JAR_AUTONS_H_
