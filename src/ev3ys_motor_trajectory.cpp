#include "ev3ys_motor_trajectory.h"
#include <cmath>
#include <algorithm>
#include "ev3ys_math.h"

using namespace std;
using namespace ev3ys;

trajectory::trajectory()
{
    setLimits(0, 0);
    makeStationary(0, 0);
}

void trajectory::setLimits(double maxVelocity, double maxAcceleration)
{
    v_max = maxVelocity;
    a_max = maxAcceleration;
}

double trajectory::getStartTime()
{
    return t0;
}

double trajectory::getEndTime()
{
    return t3;
}

double trajectory::getStartPos()
{
    return p0;
}

double trajectory::getEndPos()
{
    return p3;
}

void trajectory::reverseTrajectory()
{
    p1 = 2 * p0 - p1;
    p2 = 2 * p0 - p2;
    p3 = 2 * p0 - p3;

    v0 *= -1;
    v1 *= -1;
    a *= -1;
    d *= -1;
}

double trajectory::calcVelocity(double a, double dt)
{
    return a * dt;
}

double trajectory::calcPosition(double a, double dt)
{
    return 0.5 * a * pow(dt, 2);
}

double trajectory::calcPositionConstant(double v, double dt)
{
    return v * dt;
}

double trajectory::calcTime(double dv, double a)
{
    return dv / a;
}

void trajectory::makeStationary(double startTime, double startPosition)
{
    t0 = startTime;
    t1 = startTime;
    t2 = startTime;
    t3 = startTime;

    p0 = startPosition;
    p1 = startPosition;
    p2 = startPosition;
    p3 = startPosition;

    v0 = 0;
    v1 = 0;
    a = 0;
    d = 0;
}

void trajectory::makeTimeBased(double targetVelocity, double duration, double acceleration, double startVelocity, double endVelocity, double startPosition, double startTime)
{
    // Work with time intervals instead of absolute time.
    double dt_3_0;
    double dt_3_2;
    double dt_2_1;
    double dt_1_0;

    // Duration of the maneuver
    if (duration == DURATION_FOREVER)
    {
        // In case of forever, we set the duration to a fictitious 120 seconds.
        dt_3_0 = 120;
        // This is an infinite maneuver. (This means we'll just ignore the deceleration
        // phase when computing references later, so we keep going even after 120 seconds.)
        isForever = true;
    }
    else
    {
        // Otherwise, the interval is just the duration
        dt_3_0 = duration;
        // This is a finite maneuver
        isForever = false;
    }

    // Remember if the original user-specified maneuver was backward
    bool isBackward = targetVelocity < 0;

    // Convert user parameters into a forward maneuver to simplify computations (we negate results at the end)
    if (isBackward) 
    {
        targetVelocity *= -1;
        startVelocity *= -1;
        endVelocity *= -1;
    }

    // Limit absolute acceleration
    acceleration = min(acceleration, a_max);

    // Limit initial speed
    double possibleMax = calcVelocity(acceleration, dt_3_0);
    double absoluteMax = min(v_max, possibleMax);
    startVelocity = clamp(startVelocity, -absoluteMax, absoluteMax);
    targetVelocity = clamp(targetVelocity, -absoluteMax, absoluteMax);
    endVelocity = clamp(endVelocity, -absoluteMax, absoluteMax);

    if(startVelocity < targetVelocity)
    {
        //Initial speed is less than the target speed therefore accelerate
        a = acceleration;

        // If target speed can be reached
        if((endVelocity + startVelocity + calcVelocity(acceleration, dt_3_0)) / 2 > targetVelocity)
        {
            dt_1_0 = calcTime(targetVelocity - startVelocity, acceleration);
            v1 = targetVelocity;
        }
        // If target speed cannot be reached
        else
        {
            dt_1_0 = (dt_3_0 / 2) + (endVelocity - startVelocity) / (2 * acceleration);
            v1 = (endVelocity + startVelocity + calcVelocity(acceleration, dt_3_0)) / 2;
        }
    }
    else if(startVelocity > targetVelocity)
    {
        //Initial speed is more than the target speed therefore decelerate
        a = -acceleration;
        dt_1_0 = calcTime(startVelocity - targetVelocity, acceleration);
        v1 = targetVelocity;
    }
    else
    {
        //Initial speed is equal to the target speed therefore no acceleration
        a = 0;
        dt_1_0 = 0;
        v1 = targetVelocity;
    }

    //Deceleration phase
    d = -acceleration;
    dt_3_2 = calcTime(v1 - endVelocity, acceleration);

    // Constant speed duration
    dt_2_1 = dt_3_0 - dt_3_2 - dt_1_0;

    // Store other velocity and time values
    v0 = startVelocity;
    t0 = startTime;
    t1 = startTime + dt_1_0;
    t2 = startTime + dt_1_0 + dt_2_1;
    t3 = startTime + dt_3_0;

    // Corresponding position values
    p0 = startPosition;
    p1 = p0 + calcPositionConstant(v0, dt_1_0) + calcPosition(a, dt_1_0);
    p2 = p1 + calcPositionConstant(v1, dt_2_1);
    p3 = p2 + calcPositionConstant(v1, dt_3_2) + calcPosition(d, dt_3_2);

    // Reverse the maneuver if the original arguments imposed backward motion
    if(isBackward)
    {
        reverseTrajectory();
    }
}

void trajectory::makePositionBased(double targetVelocity, double distance, double acceleration, double startVelocity, double endVelocity, double startPosition, double startTime)
{
    // Return empty maneuver for zero angle
    if(distance == startPosition)
    {
        makeStationary(startTime, startPosition);
    }

    // Remember if the original user-specified maneuver was backward and convert user parameters into a forward maneuver to simplify computations (we negate results at the end)
    bool isBackward = false;
    if(distance < startPosition)
    {
        isBackward = true;
        distance = 2 * startPosition - distance;
        startVelocity *= -1;
        endVelocity *= -1;
    }
    else if(targetVelocity < 0)
    {
        isBackward  = true;
        targetVelocity *= -1;
        startVelocity *= -1;
        endVelocity *= -1;
    }

    // Limit absolute acceleration
    acceleration = min(acceleration, a_max);

    // In a forward maneuver, the target speed is always positive.
    targetVelocity = absolute(targetVelocity);
    targetVelocity = min(targetVelocity, v_max);

    // Limit initial speed
    startVelocity = clamp(startVelocity, -v_max, v_max);
    endVelocity = clamp(endVelocity, -v_max, v_max);

    // Limit initial speed, but evaluate square root only if necessary (usually not)
    if(startVelocity > 0 && pow(startVelocity, 2) / (2 * acceleration) > distance - startPosition)
    {
        startVelocity = sqrt(2 * acceleration * (distance - startPosition));
    }

    if (startVelocity < targetVelocity)
    {
        // Initial speed is less than the target speed therefore accelerate
        a = acceleration;

        //If target speed can be reached
        double vf = sqrt((pow(startVelocity, 2) + pow(endVelocity, 2) + (2 * acceleration * (distance - startPosition))) / 2);
        if(vf > targetVelocity)
        {
            v1 = targetVelocity;
            p1 = startPosition + (pow(v1, 2) - pow(startVelocity, 2)) / (2 * acceleration);
            p2 = distance + (pow(endVelocity, 2) - pow(v1, 2)) / (2 * acceleration);
        }
        //If target speed cannot be reached
        else
        {
            v1 = vf;
            p1 = startPosition + (pow(v1, 2) - pow(startVelocity, 2)) / (2 * acceleration);
            p2 = p1;
        }
    }
    else
    {
        // Initial speed is equal to or more than the target speed
        // Therefore decelerate towards intersection from above
        a = -acceleration;
        p1 = startPosition + (pow(startVelocity, 2) - pow(targetVelocity, 2)) / (2 * acceleration);
        v1 = targetVelocity;
        p2 = distance + (pow(endVelocity, 2) - pow(v1, 2)) / (2 * acceleration);
    }

    // Corresponding time intervals
    double dt_1_0 = calcTime(v1 - startVelocity, a);
    double dt_2_1 = p2 == p1 ? 0 : calcTime(p2 - p1, v1);
    double dt_3_2 = calcTime(v1 - endVelocity, acceleration);

    // Store other results/arguments
    v0 = startVelocity;
    p0 = startPosition;
    p3 = distance;
    t0 = startTime;
    t1 = startTime + dt_1_0;
    t2 = t1 + dt_2_1;
    t3 = t2 + dt_3_2;
    d = -acceleration;

    // Reverse the maneuver if the original arguments imposed backward motion
    if(isBackward)
    {
        reverseTrajectory();
    }

    // This is a finite maneuver
    isForever = false;
}

void trajectory::getReference(double time, double *position, double *velocity, double *acceleration)
{
    double dt = 0;
    if(time - t1 < 0)
    {
        //Acceleration Phase
        dt = time - t0;
        *velocity = v0 + calcVelocity(a, dt);
        *position = p0 + calcPositionConstant(v0, dt) + calcPosition(a, dt);
        *acceleration = a;
    }
    else if(isForever || time - t2 <= 0)
    {
        //Constant Speed Phase
        dt = time - t1;
        *velocity = v1;
        *position = p1 + calcPositionConstant(v1, dt);
        *acceleration = 0;
    }
    else if(time - t3 <= 0)
    {
        //Deceleration phase
        dt = time - t2;
        *velocity = v1 + calcVelocity(d, dt);
        *position = p2 + calcPositionConstant(v1, dt) + calcPosition(d, dt);
        *acceleration = d;
    }
    else
    {
        //Zero speed / Hold phase
        *velocity = 0;
        *position = p3;
        *acceleration = 0;
    }
}
