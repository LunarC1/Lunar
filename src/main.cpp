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

pros::adi::DigitalIn limitSwitch('F');

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

// Drive Curve
int curveA3(float input, float curveScale){
	int retVal = (powf(2.718, -(curveScale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(curveScale / 10)))) * input;
	return retVal;
}

int curveA1(float input, float curveScale){
  	int retVal = (powf(2.718, -(curveScale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(curveScale / 10)))) * input;
	return retVal;
}

// Different Autons
void driveTest(){
	chassis.setHeading(0);
    chassis.driveDist(24,0,0,127);
}
void turnTest(){
	chassis.setHeading(0);
	chassis.turnHeading(90,0,127);
}
void swingTest(){
	chassis.setHeading(0);
	chassis.lSwing(90);
	chassis.rSwing(0);
}
void odomTest(){
	chassis.setHeading(0);
}
void skills(){
	chassis.setHeading(0);
}
void test(){
	chassis.setHeading(0);
}
// Pos Red, Neg Red, Pos Blue, Neg Blue, Skills

void posRed(){

}
void negRed(){
	
}
void posBlue(){
	
}
void negBlue(){
	
}

int autonState = 0; 

void autonSelect(){
    while (1){ pros::delay(20); if(limitSwitch.get_new_press()){ autonState ++; if(autonState>5){ autonState = 0; } }
	switch(autonState){
		case 0: 
			pros::lcd::set_text(2, "Drive Test");
			break;
		case 1: 
			pros::lcd::set_text(2, "Turn Test");
			break;
		case 2: 
			pros::lcd::set_text(2, "Swing Test");
			break;
		case 3: 
			pros::lcd::set_text(2, "Odom Test");
			break;
		case 4: 
			pros::lcd::set_text(2, "Skills");
			break;
		case 5: 
			pros::lcd::set_text(2, "Overall Test");
			break;
		default: 
			pros::lcd::set_text(2, "AUTON");
			break;     
        }
    }
}

void initialize() {
	pros::lcd::initialize();
	pros::Task select(autonSelect);
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

	// Path names
	// Pos Red, Neg Red, Pos Blue, Neg Blue, Skills
	
	chassis.setHeading(0);
	chassis.driveDist(24,0,0,127);
	pros::delay(10000000);
	// chassis.turnHeading(90);
	// chassis.lSwing(90);
	// chassis.rSwing(90);
	switch(autonState){
		case 0: 
			driveTest(); 
			break;
		case 1: 
			turnTest(); 
			break;
		case 2: 
			swingTest();  
			break;
		case 3: 
			odomTest(); 
			break;
		case 4: 
			skills(); 
			break;
		case 5:
			test();
			break; 
		default: 
			break;
    }
}

void opcontrol() {
	bool mogoc = false;
	double leftCurveScale = 10; // 10 is default
	double RightCurveScale = 10; // 10 is default

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

		chassis.arcade(curveA3(leftY,leftCurveScale),curveA1(rightX,RightCurveScale)); // Arcade drive


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

		pros::delay(10);
	}
}