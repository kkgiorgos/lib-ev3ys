#pragma once

#include "ev3ys_motor_trajectory.h"

namespace ev3ys
{
    enum controlType
    {
        CONTROL_NONE,       //No control
        CONTROL_TIMED,      //Run for a given amount of time
        CONTROL_POSITION,   //Run to an angle
        CONTROL_HOLD        //Hold position
    };

    enum checkTargetType
    {
        TARGET_ALWAYS,
        TARGET_NEVER,
        TARGET_POSITION,
        TARGET_TIME
    };

    class control
    {
    private:
        double maxVelocity;
        double maxAcceleration;

        double velocityTolerance;           // Allowed deviation from target speed
        double positionTolerance;           // Allowed deviation from target before motion is considered complete
        
        double pidKp;                       // Proportional position control constant
        double pidKi;                       // Integral position control constant
        double pidKd;                       // Derivative position control constant (and proportional speed control constant)
        double error, error_i, error_d, error_old;

        double maxOutput;
        
        double dt;

        controlType type;
        trajectory trj;
        checkTargetType targetType;
        bool isOnTarget;

        void resetPID();

        bool checkTarget(double time, double position, double velocity);
    public:
        control();

        void setAbsoluteLimits(double velocity, double acceleration, double actuation);
        void setPID(double pidKp, double pidKi, double pidKd, double dt);
        void setTargetTolerance(double velocity, double position);

        void stop();
        void startPosition(double positionTarget, double velocityTarget, double acceleration, double velocityNow = 0, double velocityEnd = 0, double positionNow = 0);
        void startTime(double duration, double velocityTarget, double acceleration, checkTargetType targetType = TARGET_TIME, double velocityNow =  0, double velocityEnd = 0);
        void startHold(double positionTarget);

        bool isDone();

        double update(double timeNow, double positionNow, double velocityNow);

        double updateManual(double timeNow, double positionNow, double velocityNow, double positionTarget, double velocityTarget);
    };
}
