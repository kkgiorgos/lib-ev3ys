#include "ev3ys_motor.h"

using namespace std;
using namespace ev3cxx;

namespace ev3ys
{
    const int motor::maxMediumSpeed = 1815;
    const int motor::maxLargeSpeed = 1165;
    const int motor::maxUnregSpeedPCT = 100;
    const int motor::maxUnregSpeedDPS = 1150;
    const int motor::speedLimit = 1150;

    motor::motor(MotorPort port, bool inversed, MotorType type) : Motor(port, type)
    {
        direction = inversed ? -1 : 1;
        unregulatedDPS = false;
        if(type == MotorType::MEDIUM)
        {
            this->type = motorType::MEDIUM;
            motorController.setAbsoluteLimits(speedLimit, 8000, 100);
            motorController.setPID(2, 1.2, 0.07, 0.002);
            motorController.setTargetTolerance(50, 5);
        }
        else
        {
            this->type = motorType::LARGE;
            motorController.setAbsoluteLimits(speedLimit, 3000, 100);
            motorController.setPID(2, 1.2, 0.07, 0.002);//THESE HAVE NOT BEEN TESTED
            motorController.setTargetTolerance(50, 5);
        }
        setMode(speedMode::REGULATED);
        setAccelParams(1000);
        setStallTolerance(200, 10, 0.5);
        setSpeedLimiter(true);
        tachoCountReset = 0;
    }

    void motor::setMode(speedMode mode)
    {
        this->mode = mode;
        if(mode == UNREGULATED) 
        {
            if(unregulatedDPS)
                maxSpeed = maxUnregSpeedDPS;
            else
                maxSpeed = maxUnregSpeedPCT;
        }

        else if(type == motorType::LARGE) maxSpeed = maxLargeSpeed;
        else maxSpeed = maxMediumSpeed;
    }

    speedMode motor::getMode()
    {
        return mode;
    }

    void motor::setAccelParams(int acceleration, int startSpeed, int endSpeed)
    {
        this->acceleration = acceleration;
        this->startSpeed = startSpeed;
        this->endSpeed = endSpeed;
    }

    void motor::setStallTolerance(int speedToleranceDPS, int speedTolerancePCT, double stallTime)
    {
        this->speedToleranceDPS = speedToleranceDPS;
        this->speedTolerancePCT = speedTolerancePCT;
        this->stallTime = stallTime;
    }

    void motor::setUnregulatedDPS(bool isDPS)
    {
        unregulatedDPS = isDPS;
        setMode(UNREGULATED);
    }

    void motor::setSpeedLimiter(bool doLimit)
    {
        limitSpeed = doLimit;
    }

    int motor::getSpeedLimit()
    {
        return speedLimit;
    }

    int motor::trimSpeed(int speed)
    {
        return limitSpeed ? clamp(speed, -speedLimit, speedLimit) : speed;
    }

    int motor::dps_to_pct(int speed)
    {
        return (speed/(double)maxSpeed) * 100;
    }

    int motor::pct_to_dps(int speed)
    {
        return maxSpeed * (speed / 100.0);
    }

    int motor::getCurrentSpeed()
    {
        return direction * pct_to_dps(currentPower());
    }

    int motor::getTachoCount()
    {
        return direction * degrees();
    }

    int motor::getAbsoluteTachoCount()
    {
        return getTachoCount() + tachoCountReset;
    }

    void motor::resetTachoCount()
    {
        tachoCountReset += getTachoCount();
        resetPosition();
    }

    void motor::initialiseAbsoluteCount()
    {
        resetTachoCount();
        tachoCountReset = 0;
    }

    void motor::stop(breakMode stopMode)
    {
        setMode(mode);
        motorController.stop();
        switch(stopMode)
        {
        case breakMode::HOLD:
            t.reset();
            motorController.startHold(getTachoCount());
            while(t.secElapsed() < 0.2)
            {
                on(motorController.update(t.secElapsed(), getTachoCount(), getCurrentSpeed()));
            }
            motorController.stop();
            off(true);
            break;
        case breakMode::BRAKE:
            off(true);
            break;
        case breakMode::COAST:
            off(false);
            break;
        case breakMode::BRAKE_COAST:
            off(true);
            tslp_tsk(1);
            off(false);
            break;
        case breakMode::ZERO:
            unregulated(0);
            break;
        case breakMode::NONE:
            break;
        default:
            off(true);
            break;
        }
    }

    void motor::moveUnlimited(int speed, bool resetTacho) //UNREGULATED MODE speed in pct OTHERWISE in deg/s
    {
        setMode(mode);
        if(resetTacho) resetTachoCount();
        
        speed *= direction;

        if(mode == speedMode::UNREGULATED)
        {
            speed = (unregulatedDPS) ? dps_to_pct(speed) : speed;
            unregulated(speed);
        }
        else if(mode == speedMode::REGULATED)
        {
            speed = dps_to_pct(trimSpeed(speed));
            on(speed);
        }
        else //mode == CONTROLLED  has to be repeatedly called.  to start new maneuver resetTacho otherwise it continues the previous
        {
            if(resetTacho)
            {
                motorController.stop();
                motorController.startTime(DURATION_FOREVER, direction * speed, acceleration, checkTargetType::TARGET_NEVER, startSpeed, endSpeed);
                t.reset();
            }
            speed = direction * motorController.update(t.secElapsed(), getTachoCount(), getCurrentSpeed());
            on(speed);
        }
    }

    void motor::moveDegrees(int speed, int degrees, breakMode stopMode, bool wait)
    {
        setMode(mode);
        resetTachoCount();
        
        speed *= direction * sign(degrees);
        degrees = abs(degrees);

        speedMode prevMode = getMode();

        if(!wait) setMode(speedMode::REGULATED);

        if(mode == speedMode::UNREGULATED)
        {
            speed = (unregulatedDPS) ? dps_to_pct(speed) : speed;
            while(abs(getTachoCount()) < degrees)
            {
                unregulated(speed);
            }
        }
        else if(mode == speedMode::REGULATED)
        {
            speed = dps_to_pct(trimSpeed(speed));
            bool modeStop = wait ? false : stopMode == BRAKE ? true : false;
            onForDegrees(speed, degrees, modeStop, wait, 0);
        }
        else //mode == CONTROLLED
        {
            t.reset();
            motorController.startPosition(degrees, direction * speed, acceleration, startSpeed, endSpeed, 0);
            while(!motorController.isDone())
            {
                speed = direction * motorController.update(t.secElapsed(), getTachoCount(), getCurrentSpeed());
                on(speed);
            }
            motorController.stop();
        }
        setMode(prevMode);

        if(wait) stop(stopMode);
    }

    void motor::moveDegreesAbsolute(int speed, int degrees, breakMode stopMode)
    {
        setMode(mode);
        moveDegrees(speed, degrees - getAbsoluteTachoCount(), stopMode);
    }

    void motor::moveSeconds(int speed, double seconds, breakMode stopMode)
    {
        setMode(mode);
        resetTachoCount();
        t.reset();

        speed *= direction;

        if(mode == speedMode::UNREGULATED)
        {
            speed = (unregulatedDPS) ? dps_to_pct(speed) : speed;
            while(t.secElapsed() < seconds)
            {
                unregulated(speed);
            }
        }
        else if(mode == speedMode::REGULATED)
        {
            speed = dps_to_pct(trimSpeed(speed));
            while(t.secElapsed() < seconds)
            {
                on(speed);
            }
        }
        else //mode == CONTROLLED
        {
            motorController.startTime(seconds, direction * speed, acceleration, checkTargetType::TARGET_TIME, startSpeed, endSpeed);
            while(!motorController.isDone())
            {
                speed = direction * motorController.update(t.secElapsed(), getTachoCount(), getCurrentSpeed());
                on(speed);
            }
            motorController.stop();
        }
        stop(stopMode);
    }

    bool motor::isStalled(int speed)
    {
        if(mode == speedMode::REGULATED)
        {
            speed = (unregulatedDPS) ? dps_to_pct(speed) : speed;
            if(abs(abs(speed) - abs(getCurrentSpeed())) > speedTolerancePCT)
            {
                if(stallTimerStarted)
                {
                    if(t.secElapsed() > stallTime)
                    {
                        stallTimerStarted = false;
                        return true;
                    }
                }
                else
                {
                    stallTimerStarted = true;
                    t.reset();
                }
            }
        }
        return false;
    }

    void motor::moveUntilStalled(int speed, breakMode stopMode, double maxTimeLimit, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetTachoCount();

        speed *= direction;

        speedMode prevMode = getMode();

        if(mode == speedMode::UNREGULATED)
        {
            speed = (unregulatedDPS) ? dps_to_pct(speed) : speed;
            t.reset();
            unregulated(speed);
            while(t.secElapsed() < maxTimeLimit)
            {
                if(abs(abs(speed) - abs(getCurrentSpeed())) > speedTolerancePCT)
                {
                    t.secDelay(stallTime);
                    if(abs(abs(speed) - abs(getCurrentSpeed())) > speedTolerancePCT)break;
                }
            }
        }
        else if(mode == speedMode::REGULATED)
        {
            t.reset();
            on(dps_to_pct(trimSpeed(speed)));
            while(t.secElapsed() < maxTimeLimit)
            {
                if(abs(abs(speed) - abs(getCurrentSpeed())) > speedToleranceDPS)
                {
                    t.secDelay(stallTime);
                    if(abs(abs(speed) - abs(getCurrentSpeed())) > speedToleranceDPS)break;
                }
            }
        }
        else //mode == speedMode::CONTROLLED
        {
            speed *= direction;
            t.reset();
            timer stallTimer;
            moveUnlimited(speed, true);
            while(t.secElapsed() < maxTimeLimit)
            {
                if(abs(abs(speed) - abs(getCurrentSpeed())) > speedToleranceDPS)
                {
                    stallTimer.reset();
                    while(stallTimer.secElapsed() < stallTime)
                    {
                        moveUnlimited(speed);
                    }
                    if(abs(abs(speed) - abs(getCurrentSpeed())) > speedToleranceDPS)break;
                }
                moveUnlimited(speed);
            }
        }

        setMode(prevMode);

        stop(stopMode);
    }
}
