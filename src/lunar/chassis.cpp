#include "math.h"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lunar/chassis.hpp"
#include "pros/rtos.hpp"
#include "lunar/api.hpp"

lunar::Sensors::Sensors(pros::Imu* imu)
    : imu(imu) {}

lunar::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter, float rpm)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm) {}

lunar::Chassis::Chassis(Drivetrain drivetrain, Sensors sensors, Constraints lateralSettings, Constraints angularSettings )
    : drivetrain(drivetrain),
      sensors(sensors),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings), 
      lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, lateralSettings.settleError, lateralSettings.settleTime, lateralSettings.timeout),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, angularSettings.settleError, angularSettings.settleTime, angularSettings.timeout) {}

void lunar::Chassis::callibrate(){
    sensors.imu->reset();
    drivetrain.leftMotors->tare_position_all();
    drivetrain.rightMotors->tare_position_all();
}

void lunar::Chassis::set(float angle){
    sensors.imu->set_heading(angle);
}

void lunar::Chassis::tank(float leftVolt, float rightVolt){
    drivetrain.leftMotors->move(leftVolt);
    drivetrain.rightMotors->move(rightVolt);
}

void lunar::Chassis::arcade(float throttle, float turn){
    float leftVolt = throttle + turn;
    float rightVolt = throttle - turn;

    tank(leftVolt, rightVolt);
}

// void lunar::Chassis::driveDist(float dist){
//     driveDist(dist,lunar::Sensors::imu->heading());
// }

void lunar::Chassis::driveDist(float dist, float heading){
    lunar::Chassis::leftDist = drivetrain.leftMotors->get_position()*(36/48);
    lunar::Chassis::rightDist = drivetrain.rightMotors->get_position()*(36/48);
    float start_average_position = (lunar::Chassis::leftDist+lunar::Chassis::rightDist)/2.0;
    float average_position = start_average_position;
    while(lateralPID.is_settled() == false){
        average_position = (lunar::Chassis::leftDist+lunar::Chassis::rightDist)/2.0;
        float drive_error = dist+start_average_position-average_position;
        float heading_error = reduce_negative_180_to_180(heading);
        float drive_output = lateralPID.update(drive_error);
        float heading_output = angularPID.update(heading_error);
        drivetrain.leftMotors->move(drive_output+heading_output);
        drivetrain.rightMotors->move(drive_output-heading_output);
    }
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
}

// void lunar::Chassis::turnHeading(float angle)