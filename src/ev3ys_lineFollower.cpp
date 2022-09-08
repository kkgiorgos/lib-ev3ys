#include "ev3ys_lineFollower.h"
#include "stdio.h"

using namespace std;
using namespace ev3cxx;

namespace ev3ys
{
    lineFollower::lineFollower(int loopFrequency, chassis *driveBase, colorSensor *sensor)
    {
        bt.open();
        this->driveBase = driveBase;
        this->sensor = sensor;

        loopPeriod = 1.0/loopFrequency;

        setAccelParams(180, 0, 0);
        setSensorMode(sensorModes::WHITE_RGB);
        initializeMotionMode(speedMode::UNREGULATED);
        setPIDparams(0.6, 0.06, 6, 75);

        resetPID();
        trj.makeStationary(0, 0);
        trj.setLimits(driveBase->getTachoSpeedLimit(), 6000);
        setSingleFollowMode("S", "50");
        sensorAmount = 1;
        setErrorScaleFactor(1);
        setAlignMode(false);
    }

    lineFollower::lineFollower(int loopFrequency, chassis *driveBase, colorSensor *leftSensor, colorSensor *rightSensor)
    {
        bt.open();
        this->driveBase = driveBase;
        this->leftSensor = leftSensor;
        this->rightSensor = rightSensor;

        loopPeriod = 1.0/loopFrequency;

        setAccelParams(180, 0, 0);
        setSensorMode(sensorModes::WHITE_RGB);
        initializeMotionMode(speedMode::UNREGULATED);
        setPIDparams(0.6, 0.06, 6, 75);

        resetPID();
        trj.makeStationary(0, 0);
        trj.setLimits(driveBase->getTachoSpeedLimit(), 6000);
        setDoubleFollowMode("SL", "SR");
        sensorAmount = 2;
        setErrorScaleFactor(1);
        setAlignMode(false);
    }

    void lineFollower::setAccelParams(double acceleration, double startSpeed, double endSpeed)
    {
        this->acceleration = acceleration;
        this->startSpeed = startSpeed;
        this->endSpeed = endSpeed;
    }

    void lineFollower::setSensorMode(sensorModes mode)
    {
        this->sensorMode = mode;
    }
    
    void lineFollower::initializeMotionMode(speedMode mode)
    {
        chassisMode = driveBase->getMode();
        chassisUnregulatedDPS = driveBase->getUnregulatedDPS();
        this->motionMode = mode;
    }

    void lineFollower::setSpeedMode()
    {
        if(motionMode == speedMode::CONTROLLED)
        {
            //driveBase->setMode(speedMode::UNREGULATED);
            //driveBase->setUnregulatedDPS(true);
            driveBase->setMode(REGULATED);
        }
        else
            driveBase->setMode(motionMode);
    }

    void lineFollower::resetChassisMode()
    {
        driveBase->setMode(chassisMode);
        driveBase->setUnregulatedDPS(chassisUnregulatedDPS);
    }

    void lineFollower::setPIDparams(double kp, double ki, double kd, double speed)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        speedPIDnormalisation = speed;
    }

    void lineFollower::setErrorScaleFactor(double factor)
    {
        errorScaleFactor = factor;
    }

    void lineFollower::setDoubleFollowMode(const char *leftPos, const char *rightPos)
    {
        target = 0;
        if(leftPos[0] == 'S' && rightPos[0] == 'S')
            followMode = TWO_SENSOR;
        else if(leftPos[0] == 'S')
        {
            if(leftPos[1] == 'L')
                followMode = LEFT_SENSOR_LEFT_POSITION;
            else
                followMode = RIGHT_SENSOR_LEFT_POSITION;
            target = atoi(rightPos);
        }
        else if(rightPos[0] == 'S')
        {
            if(rightPos[1] == 'L')
                followMode = LEFT_SENSOR_RIGHT_POSITION;
            else
                followMode = RIGHT_SENSOR_RIGHT_POSITION;
            target = atoi(leftPos);
        }
        else
            followMode = NO_SENSORS;
    }

    void lineFollower::setSingleFollowMode(const char *leftPos, const char *rightPos)
    {
        if(leftPos[0] == 'S')
        {
            followMode = ONE_SENSOR_LEFT_POSITION;
            target = atoi(rightPos);
        }
        else if(rightPos[0] == 'S')
        {
            followMode = ONE_SENSOR_RIGHT_POSITION;
            target = atoi(leftPos);
        }
        else
            followMode = NO_SENSORS;
    }
    
    void lineFollower::setAlignMode(bool enable)
    {
        alignMode = enable;
    }

    void lineFollower::resetPID()
    {
        integral = 0;
        lastError = 0;
        leftSensor->resetFiltering();
        rightSensor->resetFiltering();
    }

    double lineFollower::calculateError()
    {
        double error = 0;
        double leftVal, rightVal;
        if(followMode == NO_SENSORS)
            return  0;
        if(sensorAmount == 1)
        {
            double sensorVal;
            if(sensorMode == sensorModes::REFLECTED)
                sensorVal = sensor->getReflected();
            else
                sensorVal = sensor->getRGB().white;
            

            if(followMode == followModes::ONE_SENSOR_LEFT_POSITION)
            {
                leftVal = sensorVal;
                rightVal = target;
            }
            else
            {
                rightVal = sensorVal;
                leftVal = target;
            }
            error = leftVal - rightVal;
        }
        else
        {
            if(sensorMode == sensorModes::REFLECTED)
            {
                leftVal = leftSensor->getReflected();
                rightVal = rightSensor->getReflected();
            }    
            else
            {
                leftVal = leftSensor->getRGB().white;
                rightVal = rightSensor->getRGB().white;
            }
            

            if(followMode == followModes::TWO_SENSOR)
                error = leftVal - rightVal;
            else if(followMode == followModes::LEFT_SENSOR_LEFT_POSITION)
                error = leftVal - target;
            else if(followMode == followModes::LEFT_SENSOR_RIGHT_POSITION)
                error = target - leftVal;
            else if(followMode == followModes::RIGHT_SENSOR_LEFT_POSITION)
                error = rightVal - target;
            else if(followMode == followModes::RIGHT_SENSOR_RIGHT_POSITION)
                error = target - rightVal;
        }
        return error * errorScaleFactor;
    }

    void lineFollower::runPID(double speed)
    {
        double error = calculateError();
        integral = (integral + error) / 2;
        double derivative = error - lastError;
        lastError = error;

        double result = (kp * error + ki * integral + kd * derivative) * (alignMode ? 1 : (speed / speedPIDnormalisation)); 

        double speedLeft = speed + result;
        double speedRight = speed - result;

        driveBase->tankUnlim(speedLeft, speedRight);
    }

    void lineFollower::stop(breakMode stopMode)
    {
        resetPID();
        driveBase->stop(stopMode);
        resetChassisMode();
    }

    void lineFollower::distance(double velocity, double distance, breakMode stopMode)
    {
        resetPID();
        initializeMotionMode(motionMode);
        setSpeedMode();
        t.reset();
        driveBase->resetPosition();

        if(motionMode == speedMode::CONTROLLED)
        {
            trj.makePositionBased(velocity, distance, acceleration, startSpeed, endSpeed);
            double empty;
            while(abs(driveBase->getPosition()) < distance)
            {
                periodTimer.reset();
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                velocity = driveBase->cmToTacho(velocity);
                if(velocity == 0) break;
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        else
        {
            while(abs(driveBase->getPosition()) < distance)
            {
                periodTimer.reset();
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        resetChassisMode();
        stop(stopMode);
    }

    void lineFollower::seconds(double velocity, double seconds, breakMode stopMode)
    {
        resetPID();
        initializeMotionMode(motionMode);
        setSpeedMode();
        t.reset();
        driveBase->resetPosition();

        if(motionMode == speedMode::CONTROLLED)
        {
            trj.makeTimeBased(velocity, seconds, acceleration, startSpeed, endSpeed);
            double empty;
            while(t.secElapsed() < seconds)
            {
                periodTimer.reset();
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                velocity = driveBase->cmToTacho(velocity);
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        else
        {
            while(t.secElapsed() < seconds)
            {
                periodTimer.reset();
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        resetChassisMode();
        stop(stopMode);
    }

    void lineFollower::unlimited(double velocity, bool reset)
    {
        setSpeedMode();
        if(reset)
        {
            driveBase->resetPosition();
            t.reset();
            trj.makeTimeBased(velocity, DURATION_FOREVER, acceleration, startSpeed, endSpeed);
            resetPID();
        }

        if(motionMode == speedMode::CONTROLLED)
        {
            double empty;
            trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
            velocity = driveBase->cmToTacho(velocity);
            runPID(velocity);
            periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            periodTimer.reset();
        }
        else
        {
            runPID(velocity);
            periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            periodTimer.reset();
        }

        resetChassisMode();
    }

    void lineFollower::lines(double velocity, int lines, breakMode stopMode)
    {
        resetPID();
        initializeMotionMode(motionMode);
        setSpeedMode();
        t.reset();
        driveBase->resetPosition();

        if(motionMode == speedMode::CONTROLLED)
        {
            trj.makeTimeBased(velocity, DURATION_FOREVER, acceleration, startSpeed, endSpeed);
            double empty;
            for(int i = 0; i < lines - 1; i++)
            {
                while(!(leftSensor->getLineDetected() || rightSensor->getLineDetected()))
                {
                    periodTimer.reset();
                    trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                    velocity = driveBase->cmToTacho(velocity);
                    runPID(velocity);
                    periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
                }
                while(leftSensor->getLineDetected() || rightSensor->getLineDetected())
                {
                    periodTimer.reset();
                    trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                    velocity = driveBase->cmToTacho(velocity);
                    runPID(velocity);
                    periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
                }
            }
            while(!(leftSensor->getLineDetected() || rightSensor->getLineDetected()))
            {
                periodTimer.reset();
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                velocity = driveBase->cmToTacho(velocity);
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        else
        {
            for(int i = 0; i < lines - 1; i++)
            {
                while(!(leftSensor->getLineDetected() || rightSensor->getLineDetected()))
                {
                    periodTimer.reset();
                    runPID(velocity);
                    periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
                }
                while(leftSensor->getLineDetected() || rightSensor->getLineDetected())
                {
                    periodTimer.reset();
                    runPID(velocity);
                    periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
                }
            }
            while(!(leftSensor->getLineDetected() || rightSensor->getLineDetected()))
            {
                periodTimer.reset();
                runPID(velocity);
                periodTimer.secDelay(loopPeriod - periodTimer.secElapsed());
            }
        }
        resetChassisMode();
        stop(stopMode);
    }
}
