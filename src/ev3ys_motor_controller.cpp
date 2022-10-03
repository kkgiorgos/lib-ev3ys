#include <cmath>
#include "ev3ys_motor_controller.h"
#include "ev3ys_math.h"
#include "ev3api.h"

using namespace std;
using namespace ev3ys;

control::control()
{
    type = CONTROL_NONE;
    targetType = TARGET_ALWAYS;
    isOnTarget = false;
    //Parameters for medium ev3 motor by default
    setAbsoluteLimits(1150, 6000, 100);
    setPID(2, 1.2, 0.07, 0.002);
    setTargetTolerance(50, 5);
}

void control::setAbsoluteLimits(double velocity, double acceleration, double actuation)
{
    maxVelocity = actuation;
    maxAcceleration = acceleration;
    trj.setLimits(velocity, acceleration);
}

void control::setPID(double pidKp, double pidKi, double pidKd, double dt)
{
    this->pidKp = pidKp;
    this->pidKi = pidKi;
    this->pidKd = pidKd;
    this->dt = dt;
}

void control::setTargetTolerance(double velocity, double position)
{
    positionTolerance = position;
    velocityTolerance = velocity;
}

void control::resetPID()
{
    error = 0;
    error_i = 0;
    error_d = 0;
    error_old = 0;
}

void control::stop()
{
    type = controlType::CONTROL_NONE;
    isOnTarget = true;
    targetType = checkTargetType::TARGET_ALWAYS;
    resetPID();
}

void control::startPosition(double positionTarget, double velocityTarget, double acceleration, double velocityNow, double velocityEnd, double positionNow)
{
    isOnTarget = false;
    targetType = TARGET_POSITION;

    //Compute trajectory
    trj.makePositionBased(velocityTarget, positionTarget, acceleration, velocityNow, velocityEnd, positionNow, 0);

    resetPID();

    //Set new control state
    type = controlType::CONTROL_POSITION;
}

void control::startTime(double duration, double velocityTarget, double acceleration, checkTargetType targetType, double velocityNow, double velocityEnd)
{
    isOnTarget = false;
    this->targetType = targetType;

    //Compute the trajectory
    trj.makeTimeBased(velocityTarget, duration, acceleration, velocityNow, velocityEnd, 0, 0);

    resetPID();

    //Set new control state
    type = controlType::CONTROL_TIMED;
}

void control::startHold(double positionTarget)
{
    isOnTarget = false;
    targetType = checkTargetType::TARGET_NEVER;

    //Compute new maneuver based on user argument, starting from the initial state
    trj.makeStationary(0, positionTarget);

    resetPID();

    type = controlType::CONTROL_HOLD;
}

bool control::isDone()
{
    return type == controlType::CONTROL_NONE || isOnTarget;
}

bool control::checkTarget(double time, double position, double velocity)
{
    switch(targetType)
    {
    case TARGET_ALWAYS:
        return true;
    case TARGET_POSITION:
        if(trj.getEndPos() - position > positionTolerance) return false;   //Distance to target is still bigger than the tolerance, so we are not there yet
        if(position - trj.getEndPos() > positionTolerance) return false;   //Distance past target is still bigger than the tolerance so we are too far so not there yet
        if(absolute(velocity) > velocityTolerance) return false;                //The motor is not standing still so we are not there yet
        return true; //There is nothing left to do, so we must be on target
    case TARGET_NEVER:
        return false;
    case TARGET_TIME:
        return time >= trj.getEndTime();
    default:
        return true;
    }
}

double control::update(double timeNow, double positionNow, double velocityNow)
{
    double empty;
    double result;

    //Computation of the control signal

    if(type == controlType::CONTROL_HOLD)
    {
        double target;
        trj.getReference(timeNow, &target, &empty, &empty);
        error = target - positionNow;
        error_d = (error - error_old) / dt;
        result = error * pidKp + error_i * pidKi + error_d * pidKd;
        if(absolute(result) > maxVelocity && ((error >= 0 && error_i >= 0) || (error < 0 && error_i < 0)))
        {
            error_i = error_i;
        }
        else
        {
            error_i = error_i + (error * dt);
        }
        error_old = error;
    }
    else
    {
        double positionTarget, velocityTarget, accelerationTarget;
        trj.getReference(timeNow, &positionTarget, &velocityTarget, &accelerationTarget);
        error = positionTarget - positionNow;
        error_d = velocityTarget - velocityNow;
        result = error * pidKp + error_d * pidKd;
    }

    result = clamp(result, -maxVelocity, maxVelocity);

    //Check if we are on target
    isOnTarget = checkTarget(timeNow, positionNow, velocityNow);

    tslp_tsk(dt * 1000);

    //If we are done return 0 result otherwise return the result
    if(isOnTarget) return 0;

    return result;
}

double control::updateManual(double timeNow, double positionNow, double velocityNow, double positionTarget, double velocityTarget)
{
    double result;

    //Computation of the control signal
    error = positionTarget - positionNow;
    error_d = velocityTarget - velocityNow;
    result = error * pidKp + error_d * pidKd;

    result = clamp(result, -maxVelocity, maxVelocity);

    return result;
}
