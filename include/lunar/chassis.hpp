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

struct turnHeadingParams{
    float maxspeed = 127;
    float minspeed = 0;
    bool forwards = true;
};

struct moveDistParams{
    float maxspeed = 127;
    float minspeed = 0;
    bool forwards = true;
};

class Chassis{
    public:
        Chassis(Drivetrain drivetrain, Sensors sensors, Constraints lateralSettings, Constraints angularSettings);
        
        int leftDist;
        int rightDist;

        void callibrate();
        void set(float angle);

        void tank(float leftVolt, float rightVolt);
        void arcade(float throttle, float turn);

        void driveDist(float dist, float heading);

        PID lateralPID;
        PID angularPID;

    protected:
        Constraints lateralSettings;
        Constraints angularSettings;
        Drivetrain drivetrain;
        Sensors sensors;
};

}//namespace lunar