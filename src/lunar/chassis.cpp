#include "math.h"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lunar/chassis.hpp"
#include "pros/rtos.hpp"

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

void lunar::Chassis::tank(float leftVolt, float rightVolt){
    drivetrain.leftMotors->move(leftVolt);
    drivetrain.rightMotors->move(rightVolt);
}

void lunar::Chassis::arcade(float throttle, float turn){
    float leftVolt = throttle + turn;
    float rightVolt = throttle - turn;

    tank(leftVolt, rightVolt);
}