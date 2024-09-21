#include "math.h"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lunar/chassis.hpp"
#include "pros/rtos.hpp"
#include "lunar/api.hpp"

lunar::Sensors::Sensors(pros::Imu* imu)
    : imu(imu) {}

lunar::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter, float gearRatio)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      gearRatio(gearRatio) {}

lunar::Chassis::Chassis(Drivetrain drivetrain, Sensors sensors, Constraints lateralSettings, Constraints angularSettings, Constraints swingSettings)
    : drivetrain(drivetrain),
      sensors(sensors),
      lateralSettings(lateralSettings),
      angularSettings(angularSettings), 
      swingSettings(swingSettings), 
      lateralPID(lateralSettings.kP, lateralSettings.kI, lateralSettings.kD, lateralSettings.windupRange, lateralSettings.settleError, lateralSettings.settleTime, lateralSettings.timeout),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, angularSettings.settleError, angularSettings.settleTime, angularSettings.timeout), 
      swingPID(swingSettings.kP, swingSettings.kI, swingSettings.kD, swingSettings.windupRange, swingSettings.settleError, swingSettings.settleTime, swingSettings.timeout){}

void lunar::Chassis::callibrate(){
    sensors.imu->reset();
    drivetrain.leftMotors->tare_position_all();
    drivetrain.rightMotors->tare_position_all();
}

void lunar::Chassis::setHeading(float angle){
    sensors.imu->set_heading(angle);
}

void lunar::Chassis::tank(float leftVolt, float rightVolt){
    drivetrain.leftMotors->move(leftVolt);
    drivetrain.rightMotors->move(rightVolt);
}

void lunar::Chassis::arcade(float throttle, float turn){
    float leftVolt = (throttle + turn);
    float rightVolt = (throttle - turn);
    tank(leftVolt, rightVolt);
}

void lunar::Chassis::arcadeCurve(float throttle, float turn, float lScale = 5, float rScale = 5){
    float leftVolt = ((powf(2.718, -(lScale / 10)) + powf(2.718, (fabs(lScale) - 127) / 10) * (1 - powf(2.718, -(lScale / 10))))) * (throttle + turn);
    float rightVolt = ((powf(2.718, -(rScale / 10)) + powf(2.718, (fabs(rScale) - 127) / 10) * (1 - powf(2.718, -(rScale / 10))))) * (throttle - turn);
    tank(leftVolt, rightVolt);
}

void lunar::Chassis::tankCurve(float lV, float rV, float lScale = 5, float rScale = 5){
    float leftVolt = ((powf(2.718, -(lScale / 10)) + powf(2.718, (fabs(lScale) - 127) / 10) * (1 - powf(2.718, -(lScale / 10))))) * lV;
    float rightVolt = ((powf(2.718, -(rScale / 10)) + powf(2.718, (fabs(rScale) - 127) / 10) * (1 - powf(2.718, -(rScale / 10))))) * rV;
    tank(leftVolt, rightVolt);
}

void lunar::Chassis::chain(){
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
}

void lunar::Chassis::coast(){
  drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::coast);
  drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::coast);
  drivetrain.leftMotors->brake();
  drivetrain.rightMotors->brake();
}

void lunar::Chassis::hold(bool L = 1, bool R = 1){
  if(L == 1) { drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::hold); drivetrain.leftMotors->brake(); }
  else if(R == 1) { drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::hold); drivetrain.rightMotors->brake(); }
  else { 
    drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake(); 
  }
}

void lunar::Chassis::brake(){
  drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::brake);
  drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::brake);
  drivetrain.leftMotors->brake();
  drivetrain.rightMotors->brake();
}

void lunar::Chassis::cancelMotion(std::string type){
  if(type == "BRAKE"){
    drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::brake);
    drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::brake);
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
  }
  else if (type == "COAST"){
    drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::coast);
    drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::coast); 
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake();
  }
  else if (type == "CHAIN"){
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
  }
  else if (type == "HOLD"){
    drivetrain.leftMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    drivetrain.rightMotors->set_brake_mode_all(pros::v5::MotorBrake::hold);
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->brake(); 
  }
}

void lunar::Chassis::driveDist(float dist){
    float heading  = sensors.imu->get_heading();
    driveDist(dist,heading);
}

void lunar::Chassis::driveDist(float dist, float heading, driveDistParams params, turnHeadingParams params1){
    leftDist = drivetrain.leftMotors->get_position()*drivetrain.gearRatio;
    rightDist = drivetrain.rightMotors->get_position()*drivetrain.gearRatio;
    float start_average_position = (leftDist+rightDist)/2.0;
    float average_position = start_average_position;
    while(lateralPID.is_settled() == false){
        average_position = (leftDist+rightDist)/2.0;
        float drive_error = dist+start_average_position-average_position;
        float heading_error = reduce_negative_180_to_180(heading - sensors.imu->get_heading());

        if (params.minSpeed != 0 && fabs(drive_error) < params.earlyExit) break;
        else if (params.minSpeed != 0 && fabs(heading_error) < params.earlyExit) break;

        float drive_output = lateralPID.update(drive_error);
        float heading_output = angularPID.update(heading_error);

        limit(drive_output, params.maxSpeed, params.minSpeed);
        limit(heading_output, params.maxSpeed, params.minSpeed);

        drivetrain.leftMotors->move(drive_output+heading_output);
        drivetrain.rightMotors->move(drive_output-heading_output);
        pros::delay(10);
    }
    cancelMotion("CHAIN");
}

void lunar::Chassis::turnHeading(float angle, turnHeadingParams params){
    while(angularPID.is_settled() == false){
        float error = reduce_negative_180_to_180(angle - sensors.imu->get_heading());

        if (params.minSpeed != 0 && fabs(error) < params.earlyExit) break;

        float output = angularPID.update(error);

        // Max / Min Speed
        limit(output, params.maxSpeed, params.minSpeed);
        drivetrain.leftMotors->move(-output);
        drivetrain.rightMotors->move(output);
        pros::delay(10);
    }
    cancelMotion("CHAIN");
}

void lunar::Chassis::lSwing(float angle, swingParams params){
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - sensors.imu->get_heading());
    float output = swingPID.update(error);
    limit(output, params.maxSpeed, params.minSpeed);
    drivetrain.leftMotors->move(output);
    drivetrain.rightMotors->brake();
    pros::delay(10);
  }
}

void lunar::Chassis::rSwing(float angle, swingParams params){
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - sensors.imu->get_heading());
    float output = swingPID.update(error);
    limit(output, params.maxSpeed, params.minSpeed);
    drivetrain.leftMotors->brake();
    drivetrain.rightMotors->move(output);
    pros::delay(10);
  }
}

void lunar::Chassis::diff(float vL, float vR, float timeout){
  drivetrain.leftMotors->move(vL);
  drivetrain.rightMotors->move(vR);
  pros::delay(timeout);
  cancelMotion("CHAIN");
}