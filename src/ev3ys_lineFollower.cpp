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
        setPIDparams(0.6, 0.06, 6);
        addPIDparams(0, 0.6, 0.06, 6);
        forcePIDparams(false);

        resetPID(0);
        trj.makeStationary(0, 0);
        trj.setLimits(driveBase->getTachoSpeedLimit(), 6000);
        setSingleFollowMode("S", "50");
        sensorAmount = 1;
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
        setPIDparams(0.6, 0.06, 6);
        addPIDparams(0, 0.6, 0.06, 6);
        forcePIDparams(false);

        resetPID(0);
        trj.makeStationary(0, 0);
        trj.setLimits(driveBase->getTachoSpeedLimit(), 6000);
        setDoubleFollowMode("SL", "SR");
        sensorAmount = 2;
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

    void lineFollower::addPIDparams(double velocity, double kp, double ki, double kd)
    {
        pidSpeedParams[velocity] = {kp, ki, kd};
    }

    void lineFollower::setPIDparams(double kp, double ki, double kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        forcePIDparams(true);
    }

    void lineFollower::forcePIDparams(bool doForce)
    {
        forcedParams = doForce;
    }

    void lineFollower::setDoubleFollowMode(const char *leftPos, const char *rightPos)
    {
        target = 0;
        if(leftPos[0] == 'S' && rightPos[0] == 'S')
        {
            followMode = TWO_SENSOR;
            scaling = 1;
        }
        else if(leftPos[0] == 'S')
        {
            if(leftPos[1] == 'L')
                followMode = LEFT_SENSOR_LEFT_POSITION;
            else
                followMode = RIGHT_SENSOR_LEFT_POSITION;
            target = atoi(rightPos);
            scaling = 100.0 / min(100 - target, target);
        }
        else if(rightPos[0] == 'S')
        {
            if(rightPos[1] == 'L')
                followMode = LEFT_SENSOR_RIGHT_POSITION;
            else
                followMode = RIGHT_SENSOR_RIGHT_POSITION;
            target = atoi(leftPos);
            scaling = 100.0 / min(100 - target, target);
        }
        else
        {
            followMode = NO_SENSORS;
            scaling = 0;
        }
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
        scaling = 1;
    }

    void lineFollower::resetPID(double velocity)
    {
        integral = 0;
        lastError = 0;
        leftSensor->resetFiltering();
        rightSensor->resetFiltering();
        if(!forcedParams)
        {
            map<double, PID_params>::iterator params;
            params = pidSpeedParams.find(velocity);
            if(params != pidSpeedParams.end())
            {
                kp = params->second.Kp;
                ki = params->second.Ki;
                kd = params->second.Kd;
            }
            else
            {
                kp = pidSpeedParams[0].Kp;
                ki = pidSpeedParams[0].Ki;
                kd = pidSpeedParams[0].Kd;
            }
        }
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
            lineDetected = sensor->getLineDetected();
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
            {
                error = leftVal - rightVal;
                lineDetected = leftSensor->getLineDetected() || rightSensor->getLineDetected();
            }
            else if(followMode == followModes::LEFT_SENSOR_LEFT_POSITION)
            {
                error = leftVal - target;
                lineDetected = rightSensor->getLineDetected();
            }
            else if(followMode == followModes::LEFT_SENSOR_RIGHT_POSITION)
            {
                error = target - leftVal;
                lineDetected = rightSensor->getLineDetected();
            }
            else if(followMode == followModes::RIGHT_SENSOR_LEFT_POSITION)
            {
                error = rightVal - target;
                lineDetected = leftSensor->getLineDetected();
            }
            else if(followMode == followModes::RIGHT_SENSOR_RIGHT_POSITION)
            {
                error = target - rightVal;
                lineDetected = leftSensor->getLineDetected();
            }
        }
        return error;
    }

    void lineFollower::runPID(double speed)
    {
        double error = calculateError();
        integral = (integral + error) / 2;
        double derivative = error - lastError;
        lastError = error;

        double result = (kp * error + ki * integral + kd * derivative) * scaling; 

        double speedLeft = speed + result;
        double speedRight = speed - result;

        driveBase->tankUnlim(speedLeft, speedRight);
    }

    void lineFollower::stop(breakMode stopMode)
    {
        resetPID(0);
        driveBase->stop(stopMode);
        resetChassisMode();
    }

    void lineFollower::distance(double velocity, double distance, breakMode stopMode)
    {
        resetPID(velocity);
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
                tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            }
        }
        else
        {
            velocity = driveBase->cmToTacho(velocity);
            while(abs(driveBase->getPosition()) < distance)
            {
                periodTimer.reset();
                runPID(velocity);
                tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            }
        }
        resetChassisMode();
        stop(stopMode);
    }

    void lineFollower::seconds(double velocity, double seconds, breakMode stopMode)
    {
        resetPID(velocity);
        initializeMotionMode(motionMode);
        setSpeedMode();
        t.reset();
        driveBase->resetPosition();

        if(motionMode == speedMode::CONTROLLED)
        {
            trj.makeTimeBased(velocity, seconds, acceleration, startSpeed, endSpeed);
            double empty;
            int counter = 0;
            while(t.secElapsed() < seconds)
            {
                periodTimer.reset();
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                velocity = driveBase->cmToTacho(velocity);
                runPID(velocity);
                counter++;
                tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            }
            display.resetScreen();
            display.format("Freq:\n%  \n") %(counter/t.secElapsed());
        }
        else
        {
            velocity = driveBase->cmToTacho(velocity);
            int counter = 0;
            while(t.secElapsed() < seconds)
            {
                periodTimer.reset();
                runPID(velocity);
                counter++;
                tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            }
            display.resetScreen();
            display.format("Freq:\n%  \n") %(counter/t.secElapsed());
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
            resetPID(velocity);
        }

        if(motionMode == speedMode::CONTROLLED)
        {
            double empty;
            trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
            velocity = driveBase->cmToTacho(velocity);
            runPID(velocity);
            tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            periodTimer.reset();
        }
        else
        {
            velocity = driveBase->cmToTacho(velocity);
            runPID(velocity);
            tslp_tsk((loopPeriod - periodTimer.secElapsed()) * 1000);
            periodTimer.reset();
        }

        resetChassisMode();
    }

    void lineFollower::lines(double velocity, int lines, breakMode stopMode, double distanceBeforeLine, double lineSkipDistance, bool addFinalLineSkip)
    {
        initializeMotionMode(motionMode);
        setSpeedMode();
        double positionReference = 0;

        unlimited(velocity, true);

        if(motionMode == speedMode::CONTROLLED)
        {
            trj.makeTimeBased(velocity, DURATION_FOREVER, acceleration, startSpeed, endSpeed);
            double empty;
            for(int i = 0; i < lines - 1; i++)
            {
                //Follow line for pre line distance
                positionReference = driveBase->getPosition();
                while((driveBase->getPosition() - positionReference) < distanceBeforeLine)
                {
                    unlimited(velocity);
                }
                //Follow line until intersect
                leftSensor->resetFiltering();
                rightSensor->resetFiltering();
                lineDetected = false;
                while(!lineDetected)
                {
                    unlimited(velocity);
                }
                //Skip intersect
                positionReference = driveBase->getPosition();
                followModes prevMode = followMode;
                followMode = NO_SENSORS;
                while((driveBase->getPosition() - positionReference) < lineSkipDistance)
                {
                    unlimited(velocity);
                }
                followMode = prevMode;
            }
            positionReference = driveBase->getPosition();
            while((driveBase->getPosition() - positionReference) < distanceBeforeLine)
            {
                unlimited(velocity);
            }
            leftSensor->resetFiltering();
            rightSensor->resetFiltering();
            lineDetected = false;
            while(!lineDetected)
            {
                unlimited(velocity);
            }
            if(addFinalLineSkip)
            {
                positionReference = driveBase->getPosition();
                followModes prevMode = followMode;
                followMode = NO_SENSORS;
                while((driveBase->getPosition() - positionReference) < lineSkipDistance)
                {
                    unlimited(velocity);
                }
                followMode = prevMode;
            }
        }
        else
        {
            for(int i = 0; i < lines - 1; i++)
            {
                //Follow line for pre line distance
                positionReference = driveBase->getPosition();
                while((driveBase->getPosition() - positionReference) < distanceBeforeLine)
                {
                    unlimited(velocity);
                }
                //Follow line until intersect
                leftSensor->resetFiltering();
                rightSensor->resetFiltering();
                lineDetected = false;
                while(!lineDetected)
                {
                    unlimited(velocity);
                }
                //Skip intersect
                positionReference = driveBase->getPosition();
                followModes prevMode = followMode;
                followMode = NO_SENSORS;
                while((driveBase->getPosition() - positionReference) < lineSkipDistance)
                {
                    unlimited(velocity);
                }
                followMode = prevMode;
            }
            positionReference = driveBase->getPosition();
            while((driveBase->getPosition() - positionReference) < distanceBeforeLine)
            {
                unlimited(velocity);
            }
            leftSensor->resetFiltering();
            rightSensor->resetFiltering();
            lineDetected = false;
            while(!lineDetected)
            {
                unlimited(velocity);
            }
            if(addFinalLineSkip)
            {
                positionReference = driveBase->getPosition();
                followModes prevMode = followMode;
                followMode = NO_SENSORS;
                while((driveBase->getPosition() - positionReference) < lineSkipDistance)
                {
                    unlimited(velocity);
                }
                followMode = prevMode;
            }
        }

        resetChassisMode();
        stop(stopMode);
    }
}
