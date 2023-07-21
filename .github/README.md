| WARNING: This fork is now in a theoretically functional state but has not been tested extensively. If you find any bugs, feel free to make a bug report (or submit a pull request, since I am still learning c++ and can't guarantee how actively I'll contribute to this). |
| --- |

# JAR-Template
A simple and powerful VexCode Pro V5 library featuring PID, odometry, and custom motion algorithms. 

## Installation
1. Download the project by downloading the "JAR-Template-Pros.zip" file [here](https://github.com/j-ace-svg/JAR-Template-Pros/releases/latest).

2. Import the project in your PROS project by copying in all files from the JAR src/ and include/ directories to those of your project (it is recommended to back up your main.cpp and main.h files first).

3. First configure your robot using the graphical configurer, then follow the instructions in main.cpp to configure the template to your robot specs.

4. Check out the <a href="https://jacksonarearobotics.github.io/JAR-Template/" target="_blank">documentation</a> for more detailed tutorials and information on using the template (this fork is in no way endorsed by the original but the docs should be the same).

## Usage
1. To make a new auton function, you must both write the function in autons.cpp and a declaration of the function in autons.h. The example autons show how this can be done. 
2. In order to select autons from the Brain, you must add each to the pre_auton loop and the autonomous function, both in main.cpp. Again, the example autons show how to do this.
3. The PID gains are tuned to a six motor 360RPM drivetrain. You may wish to tune the PID gains to your robot. The gains are found in autons.cpp in the function default_constants(). Each set of constants is max voltage, then kP, then kI, then kD, then startI. StartI is the maximum error at which the I term begins to wind up. For turning,  this value is best set at 15 degrees.
4. You can use a multitude of functions to control your drivebase. Every function will be prefixed with "chassis." and followed by the function name. If you want to drive to the point (36,36), then call the method "chassis.drive_to_point(36, 36);".

#### Original Credits
Developed by Josh, from 2775V of Jackson Area Robotics. Big thanks to Javier, Mikey, Ryan, Leo, JerryLum, and Nick (QUEEN) from VTOW and JPearman from the Vex Forum, whose assistance made this project possible.

#### Fork credits
Fork developed by John Montgomery, consulting programmer on a different robotics team. Most credit applies to the original coders, I just swapped out function calls and kept the api compatible with itself.

If you're interested in helping, Pull Requests and any sorts of technical suggestions are very much welcome. As little as I seem like I know, I know even less.
