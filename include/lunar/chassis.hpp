#pragma once

#include "pros/rtos.hpp"
#include "pros/imu.hpp"

#include "lunar/PID.hpp"
#include "pros/motor_group.hpp"

#include <string>

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
    float minSpeed = 0;
    float maxSpeed = 127;
    float earlyExit = 0;
    bool forwards = true;
};

struct driveDistParams{
    float minSpeed = 0;
    float maxSpeed = 127;
    float earlyExit = 0;
    bool forwards = true;
};

struct swingParams{
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExit = 0;
    bool forwards = true;
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
        void arcadeCurve(float throttle, float turn, float lScale, float rScale);

        void chain();
        void coast();
        void hold(bool L, bool R);
        void brake();
        void cancelMotion(std::string type);

        void driveDist(float dist, float heading, driveDistParams params = {}, turnHeadingParams params1 = {});
        void turnHeading(float heading, turnHeadingParams params = {});
        void lSwing(float angle);
        void rSwing(float angle);

        void diff(float vL, float vR, float timeout);

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