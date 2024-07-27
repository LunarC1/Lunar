#include "main.h"
#include "lunar/api.hpp" 

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Imu imu(10);
pros::Motor intakebot(8);
pros::Motor intaketop(-7);

pros::adi::DigitalOut mogo('A');

lunar::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.8, // 10 inch track width
                              3.25,
							  450
);

lunar::Sensors sensors(&imu // inertial sensor
);

// lateral motion controller
lunar::Constraints lateralController(10, // proportional gain (kP)
									 0, // integral gain (kI)
									 3, // derivative gain (kD)
									 3, // anti windup
									 1, // error range
									 100, // error timeout in milliseconds
									 3000 // total timeout
);

// angular motion controller
lunar::Constraints angularController(2, // proportional gain (kP)
									 0, // integral gain (kI)
									 10, // derivative gain (kD)
									 3, // anti windup
									 1, // error range
									 100, // error timeout in milliseconds
									 3000 // total timeout
);

lunar::Chassis chassis(drivetrain, sensors, lateralController, angularController);

void initialize() {
	pros::lcd::initialize();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.set(0);
	chassis.driveDist(24,0);
}

void opcontrol() {
	bool mogoc = false;

	while(1){
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY,rightX);

		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            mogoc = !mogoc; 
            mogo.set_value(mogoc);
        }
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intaketop.move(127);
		else intaketop.move(0);

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intakebot.move(127);
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) intakebot.move(-127);
		else intakebot.move(0);
	}
}