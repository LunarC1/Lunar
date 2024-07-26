#pragma once

#include "pros/rtos.hpp"
#include "pros/imu.hpp"

#include "lunar/PID.hpp"
#include "pros/motor_group.hpp"


namespace lunar{

class Sensors{
    public:
        Sensors(pros::Imu* imu);
        pros::Imu* imu;
};

class Constraints{
    public:
        Constraints(float kP, float kI, float kD, float windupRange, float settleError, float settleTime, float timeout)
            : kP(kP),
              kI(kI),
              kD(kD),
              windupRange(windupRange),
              settleError(settleError),
              settleTime(settleTime),
              timeout(timeout) {}

        float kP;
        float kI;
        float kD;
        float windupRange;
        float settleError;
        float settleTime;
        float timeout;
};

class Drivetrain{
    public:
        Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth, float wheelDiameter,
                float rpm);
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors; 
        float trackWidth;
        float wheelDiameter;
        float rpm;
};

class Chassis{
    public:
        Chassis(Drivetrain drivetrain, Sensors sensors, Constraints lateralSettings, Constraints angularSettings);
        
        void tank(float leftVolt, float rightVolt);
        void arcade(float throttle, float turn);

        PID lateralPID;
        PID angularPID;

    protected:
        Constraints lateralSettings;
        Constraints angularSettings;
        Drivetrain drivetrain;
        Sensors sensors;
};

}//namespace lunar