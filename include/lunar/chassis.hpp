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
                float gearRatio);
        pros::MotorGroup* leftMotors;
        pros::MotorGroup* rightMotors; 
        float trackWidth;
        float wheelDiameter;
        float gearRatio;
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

struct swingParams{
    float maxspeed = 127;
    float minspeed = 0;
};

class Chassis{
    public:
        Chassis(Drivetrain drivetrain, Sensors sensors, Constraints lateralSettings, Constraints angularSettings, Constraints swingSettings);
        
        int leftDist;
        int rightDist;

        void callibrate();
        void setHeading(float angle);

        void tank(float leftVolt, float rightVolt);
        void arcade(float throttle, float turn);

        void driveDist(float dist, float heading, float minspeed, float maxspeed);
        void turnHeading(float heading, float minspeed, float maxspeed);
        void lSwing(float angle);
        void rSwing(float angle);

        void diff(float vL, float vR, float timeout);

        void chain();
        // void coast();
        void hold();

        PID lateralPID;
        PID angularPID;
        PID swingPID;

    protected:
        Constraints lateralSettings;
        Constraints angularSettings;
        Constraints swingSettings;
        Drivetrain drivetrain;
        Sensors sensors;
};

}//namespace lunar