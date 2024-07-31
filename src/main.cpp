#include "main.h"
#include "lunar/api.hpp" 

/*	 /$$$$$$  /$$$$$$$$ /$$$$$$$$ /$$   /$$ /$$$$$$$ 
	/$$__  $$| $$_____/|__  $$__/| $$  | $$| $$__  $$
   | $$  \__/| $$         | $$   | $$  | $$| $$  \ $$
   |  $$$$$$ | $$$$$      | $$   | $$  | $$| $$$$$$$/
	\____  $$| $$__/      | $$   | $$  | $$| $$____/ 
	/$$  \ $$| $$         | $$   | $$  | $$| $$      
   |  $$$$$$/| $$$$$$$$   | $$   |  $$$$$$/| $$      
	\______/ |________/   |__/    \______/ |__/      */

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-13, -14, -15}, pros::MotorGearset::blue); // Left motor group - ports 13 (reversed), 14 (reversed), 15 (reversed)
pros::MotorGroup rightMotors({16, 17, 18}, pros::MotorGearset::blue); // Right motor group - ports 16, 17, 18

pros::Imu imu(12);
pros::Motor intakebot(8);
pros::Motor intaketop(-7);

pros::adi::DigitalOut mogo('A');

lunar::Drivetrain drivetrain(&leftMotors, // Left motor group
                              &rightMotors, // Right motor group
                              12.8, // Track width
                              3.25, // Wheel diameter
							  (36/48) // Gear ratio
);

lunar::Sensors sensors(&imu // inertial sensor
);

// lateral motion controller
lunar::Constraints lateralController(3, // proportional gain (kP)
									 0, // integral gain (kI)
									 0, // derivative gain (kD)
									 3, // anti windup
									 1, // error range
									 100, // error timeout in milliseconds
									 1000 // total timeout in milliseconds
);

// angular motion controller
lunar::Constraints angularController(1, // proportional gain (kP)
									 0, // integral gain (kI)
									 0, // derivative gain (kD)
									 3, // anti windup
									 1, // error range
									 100, // error timeout in milliseconds
									 2000 // total timeout in milliseconds
);

// swing motion controller
lunar::Constraints swingController(3, // proportional gain (kP)
								   0, // integral gain (kI)
								   0, // derivative gain (kD)
								   3, // anti windup
								   1, // error range
								   100, // error timeout in milliseconds
								   2000 // total timeout in milliseconds
);

lunar::Chassis chassis(drivetrain, sensors, lateralController, angularController, swingController);


void driveTest(){
	chassis.setHeading(0);
    chassis.driveDist(24,0);
}
void turnTest(){
	chassis.setHeading(0);
	chassis.turnHeading(90);
}
void swingTest(){
	chassis.setHeading(0);
	chassis.lSwing(90);
	chassis.rSwing(0);
}
void odomTest(){
	chassis.setHeading(0);
}


void initialize() {
	pros::lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {

/*    /$$$$$$  /$$   /$$ /$$$$$$$$ /$$$$$$  /$$   /$$
	 /$$__  $$| $$  | $$|__  $$__//$$__  $$| $$$  |$$
	| $$  \ $$| $$  | $$   | $$  | $$  \ $$| $$$$| $$
	| $$$$$$$$| $$  | $$   | $$  | $$  | $$| $$ $$ $$
	| $$__  $$| $$  | $$   | $$  | $$  | $$| $$  $$$$
	| $$  | $$| $$  | $$   | $$  | $$  | $$| $$\  $$$
	| $$  | $$|  $$$$$$/   | $$  |  $$$$$$/| $$ \  $$
	|__/  |__/ \______/    |__/   \______/ |__/  \__/    */  
                                                     
	chassis.setHeading(0);
	chassis.driveDist(24,0);
	// chassis.turnHeading(90);
	// chassis.lSwing(90);
	// chassis.rSwing(90);
}

void opcontrol() {
	bool mogoc = false;

	while(1){

/*   /$$$$$$$  /$$$$$$$  /$$$$$$ /$$    /$$ /$$$$$$$$ /$$$$$$$ 
	| $$__  $$| $$__  $$|_  $$_/| $$   | $$| $$_____/| $$__  $$
	| $$  \ $$| $$  \ $$  | $$  | $$   | $$| $$      | $$  \ $$
	| $$  | $$| $$$$$$$/  | $$  |  $$ / $$/| $$$$$   | $$$$$$$/
	| $$  | $$| $$__  $$  | $$   \  $$ $$/ | $$__/   | $$__  $$
	| $$  | $$| $$  \ $$  | $$    \  $$$/  | $$      | $$  \ $$
	| $$$$$$$/| $$  | $$ /$$$$$$   \  $/   | $$$$$$$$| $$  | $$
	|_______/ |__/  |__/|______/    \_/    |________/|__/  |__/    */
                                                           
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY,rightX);


/*   /$$$$$$$  /$$$$$$  /$$$$$$  /$$$$$$$$ /$$$$$$  /$$   /$$
	| $$__  $$|_  $$_/ /$$__  $$|__  $$__//$$__  $$| $$$ | $$
	| $$  \ $$  | $$  | $$  \__/   | $$  | $$  \ $$| $$$$| $$
	| $$$$$$$/  | $$  |  $$$$$$    | $$  | $$  | $$| $$ $$ $$
	| $$____/   | $$   \____  $$   | $$  | $$  | $$| $$  $$$$
	| $$        | $$   /$$  \ $$   | $$  | $$  | $$| $$\  $$$
	| $$       /$$$$$$|  $$$$$$/   | $$  |  $$$$$$/| $$ \  $$
	|__/      |______/ \______/    |__/   \______/ |__/  \__/    */
                                                         

		// Mogo clamp
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            mogoc = !mogoc; 
            mogo.set_value(mogoc);
        }

		// Intake Hook

		// 
		
/*   /$$      /$$  /$$$$$$  /$$$$$$$$ /$$$$$$  /$$$$$$$ 
	| $$$    /$$$ /$$__  $$|__  $$__//$$__  $$| $$__  $$
	| $$$$  /$$$$| $$  \ $$   | $$  | $$  \ $$| $$  \ $$
	| $$ $$/$$ $$| $$  | $$   | $$  | $$  | $$| $$$$$$$/
	| $$  $$$| $$| $$  | $$   | $$  | $$  | $$| $$__  $$
	| $$\  $ | $$| $$  | $$   | $$  | $$  | $$| $$  \ $$
	| $$ \/  | $$|  $$$$$$/   | $$  |  $$$$$$/| $$  | $$
	|__/     |__/ \______/    |__/   \______/ |__/  |__/    */
                                                    
		// Hook Motor
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intaketop.move(127);
		else intaketop.move(0);

		// Flexwheel Motor
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intakebot.move(127);
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intakebot.move(-127);
		else intakebot.move(0);

		// Wall Stakes Motor


	}
}