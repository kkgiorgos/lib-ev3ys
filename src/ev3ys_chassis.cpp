#include "ev3ys_chassis.h"

using namespace std;
using namespace ev3cxx;

namespace ev3ys
{
    chassis::chassis(motor *leftMotor, motor *rightMotor, double wheelDiameter, double axleLength, double KpRegular, double KpArc)
    {
        bt.open();
        wheelCircumference = wheelDiameter * MATH_PI;
        chassisRadius = axleLength / 2;
        this->axleLength = axleLength;
        this->leftMotor = leftMotor;
        this->rightMotor = rightMotor;

        this->Kp = KpRegular;
        this->KpRegular = KpRegular;
        this->KpArc = KpArc;

        setMode(speedMode::CONTROLLED);
        this->leftMotor->setMode(speedMode::REGULATED);
        this->rightMotor->setMode(speedMode::REGULATED);

        wheelSpeeds.left = 0;
        wheelSpeeds.right = 0;

        setLinearAccelParams(180, 0, 0);
        setAngularAccelParams(800, 0, 0);

        trj.setLimits(getTachoSpeedLimit(), 6000);

        distanceController.setTargetTolerance(2.5, 0.05);
        distanceController.setAbsoluteLimits(62.6, 326.5, getTachoSpeedLimit());
        distanceController.setPID(460, 276, 16.1, 0.001);

        //headingController.setTargetTolerance(16, 0.5);
        headingController.setTargetTolerance(5, 3);
        headingController.setAbsoluteLimits(410, 2139, getTachoSpeedLimit());
        headingController.setPID(66, 40, 2.5, 0.001);

        setStallTolerance(10, 200, 6, 40, 0.5);
        unregulatedDPS = false;
    }

    void chassis::setMode(speedMode mode)
    {
        this->mode = mode;
        if(mode == speedMode::CONTROLLED)
        {
            leftMotor->setMode(speedMode::REGULATED);
            rightMotor->setMode(speedMode::REGULATED);
        }
        else
        {
            leftMotor->setMode(mode);
            rightMotor->setMode(mode);
            if(mode == UNREGULATED)
            {
                leftMotor->setUnregulatedDPS(unregulatedDPS);
                rightMotor->setUnregulatedDPS(unregulatedDPS);
            }
        }
    }

    speedMode chassis::getMode()
    {
        return mode;
    }

    int chassis::getTachoSpeedLimit()
    {
        return leftMotor->getSpeedLimit();
    }

    void chassis::setLinearAccelParams(double acceleration, double startSpeed, double endSpeed)
    {
        linearAcceleration = acceleration;
        startLinearVelocity = startSpeed;
        endLinearVelocity = endSpeed;
    }

    void chassis::setAngularAccelParams(double acceleration, double startSpeed, double endSpeed)
    {
        angularAcceleration = acceleration;
        startAngularVelocity = startSpeed;
        endAngularVelocity = endSpeed;
    }

    void chassis::setStallTolerance(int speedTolerancePCT, int speedToleranceDPS, double speedToleranceLinear, double speedToleranceAngular, double stallTime)
    {
        this->stallTime = stallTime;
        this->speedTolerancePCT = speedTolerancePCT;
        this->speedToleranceDPS = speedToleranceDPS;
        this->speedToleranceLinear = speedToleranceLinear;
        this->speedToleranceAngular = speedToleranceAngular;
    }

    void chassis::setUnregulatedDPS(bool isDPS)
    {
        unregulatedDPS = isDPS;
        leftMotor->setUnregulatedDPS(isDPS);
        rightMotor->setUnregulatedDPS(isDPS);
    }

    double chassis::tachoToCm(double tacho)
    {
        return (tacho / 360) * wheelCircumference;
    }

    double chassis::cmToTacho(double cm)
    {
        return (cm * 360) / wheelCircumference;
    }

    double chassis::angularToTacho(double angular)
    {
        return cmToTacho(chassisRadius * degToRad(angular));
    }

    double chassis::calculateLinear(double leftTacho, double rightTacho)
    {
        return (tachoToCm(leftTacho) + tachoToCm(rightTacho)) / 2;
    }

    double chassis::calculateAngular(double leftTacho, double rightTacho)
    {
        return radToDeg((tachoToCm(leftTacho) - tachoToCm(rightTacho)) / axleLength);
    }

    double chassis::getLinearVelocity()
    {
        int leftSpeed = leftMotor->getCurrentSpeed();
        int rightSpeed = rightMotor->getCurrentSpeed();

        return calculateLinear(leftSpeed, rightSpeed);
    }

    double chassis::getAngularVelocity()
    {
        int leftSpeed = leftMotor->getCurrentSpeed();
        int rightSpeed = rightMotor->getCurrentSpeed();

        return calculateAngular(leftSpeed, rightSpeed);
    }

    double chassis::getPosition()
    {
        int leftTacho = leftMotor->getTachoCount();
        int rightTacho = rightMotor->getTachoCount();

        return calculateLinear(leftTacho, rightTacho);
    }

    double chassis::getAngle()
    {
        int leftTacho = leftMotor->getTachoCount();
        int rightTacho = rightMotor->getTachoCount();

        return calculateAngular(leftTacho, rightTacho);
    }

    double chassis::getKp()
    {
        return KpRegular;
    }

    bool chassis::getUnregulatedDPS()
    {
        return unregulatedDPS;
    }

    void chassis::resetPosition()
    {
        leftMotor->resetTachoCount();
        rightMotor->resetTachoCount();
    }

    void chassis::stop(breakMode stopMode)
    {
        setMode(mode);
        distanceController.stop();
        headingController.stop();
        if(stopMode == breakMode::HOLD)
        {
            t.reset();
            distanceController.startHold(getPosition());
            headingController.startHold(getAngle());
            while(t.secElapsed() < 0.2)
            {
                actuateControlledExternal(t.secElapsed());
            }
            distanceController.stop();
            headingController.stop();
            leftMotor->stop(breakMode::BRAKE);
            rightMotor->stop(breakMode::BRAKE);
        }
        else if(stopMode == breakMode::BRAKE_COAST)
        {
            leftMotor->stop(breakMode::BRAKE);
            rightMotor->stop(breakMode::BRAKE);
            tslp_tsk(1);
            leftMotor->stop(breakMode::COAST);
            rightMotor->stop(breakMode::COAST);
        }
        else
        {
            leftMotor->stop(stopMode);
            rightMotor->stop(stopMode);
        }
    }

    void chassis::actuateMotors(double leftSpeed, double rightSpeed)
    {
        leftMotor->moveUnlimited(leftSpeed);
        rightMotor->moveUnlimited(rightSpeed);
    }

    bool chassis::actuateControlledExternal(double time)
    {
        double leftTacho, rightTacho, leftVelocity, rightVelocity, distanceVelocity, headingVelocity;
        leftTacho = leftMotor->getTachoCount();
        rightTacho = rightMotor->getTachoCount();
        leftVelocity = leftMotor->getCurrentSpeed();
        rightVelocity = rightMotor->getCurrentSpeed();
        distanceVelocity = distanceController.update(time, calculateLinear(leftTacho, rightTacho), calculateLinear(leftVelocity, rightVelocity));
        headingVelocity = headingController.update(time, calculateAngular(leftTacho, rightTacho), calculateAngular(leftVelocity, rightVelocity));
        leftMotor->moveUnlimited(distanceVelocity + headingVelocity);
        rightMotor->moveUnlimited(distanceVelocity - headingVelocity);
        return headingVelocity == 0;
    }

    void chassis::actuateControlled(double leftSpeed, double rightSpeed)
    {
        double error, result, leftResult, rightResult;
        error = rightSpeed * leftMotor->getTachoCount() - leftSpeed * rightMotor->getTachoCount();
        result = error * Kp;
        leftResult = leftSpeed - sign(rightSpeed) * result;
        rightResult = rightSpeed + sign(leftSpeed) * result;
        leftMotor->moveUnlimited(leftResult);
        rightMotor->moveUnlimited(rightResult);
        tslp_tsk(1);
    }

    void chassis::actuateKinematically(double linearVelocity, double angularVelocity)
    {
        double distanceVelocity = cmToTacho(linearVelocity);
        double headingVelocity = angularToTacho(angularVelocity);
        leftMotor->moveUnlimited(distanceVelocity + headingVelocity);
        rightMotor->moveUnlimited(distanceVelocity - headingVelocity);
    }

    double chassis::calculateArcWheelDistances(double center, double angle, wheels wheel)
    {
        return abs(2 * MATH_PI * angle * ((chassisRadius * wheel) - center) / wheelCircumference);
    }

    speeds chassis::calculateArcSpeeds(double velocity, double leftDistance, double rightDistance, double center, double angle)
    {
        if(rightDistance > leftDistance)
        {
            wheelSpeeds.right = velocity;
            wheelSpeeds.left = velocity * (leftDistance / rightDistance);
        }
        else
        {
            wheelSpeeds.left = velocity;
            wheelSpeeds.right = velocity * (rightDistance / leftDistance);
        }

        if(0 <= center && center <= chassisRadius)
        {
            if(angle < 0)
                wheelSpeeds.left *= -1;
            else
                wheelSpeeds.right *= -1;
        }
        else if(-1 * chassisRadius <= center && center <= 0)
        {
            if(angle < 0)
                wheelSpeeds.right *= -1;
            else
                wheelSpeeds.left *= -1;
        }
        else if(angle < 0)
        {
            wheelSpeeds.left *= -1;
            wheelSpeeds.right *= -1;
        }
        return wheelSpeeds;
    }

    //Position Based
    void chassis::tank(int leftSpeed, int rightSpeed, int degrees, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        leftSpeed *= sign(degrees);
        rightSpeed *= sign(degrees);

        degrees = abs(degrees);

        while((abs(leftMotor->getTachoCount()) + abs(rightMotor->getTachoCount())) / 2 < degrees)
        {
            actuateMotors(leftSpeed, rightSpeed);
        }

        stop(stopMode);
    }

    void chassis::straight(double velocity, double distance, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        velocity *= sign(distance);
        distance = abs(distance);

        t.reset();

        trj.makePositionBased(velocity, distance, linearAcceleration, startLinearVelocity, endLinearVelocity);

        double empty;
        Kp = KpRegular;
        while(abs(getPosition()) < distance)
        {
            trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
            velocity = cmToTacho(velocity);
            if(velocity == 0) break;
            actuateControlled(velocity, velocity);
        }

        stop(stopMode);
    }

    void chassis::turn(double angularVelocity, double angle, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        angularVelocity *= sign(angle);
        angle = abs(angle);
    
        t.reset();
        distanceController.startHold(0);
        headingController.startPosition(angle, angularVelocity, angularAcceleration, startAngularVelocity, endAngularVelocity);

        Kp = KpRegular;
        while(abs(getAngle()) < angle)
        {
            if(actuateControlledExternal(t.secElapsed()))break;
        }

        stop(stopMode);
    }

    void chassis::arc(double velocity, double angle, double arcCenter, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        velocity *= sign(angle);
        angle = abs(angle);

        velocity = clamp(velocity, -getTachoSpeedLimit(), getTachoSpeedLimit());

        t.reset();

        double leftDistance = calculateArcWheelDistances(arcCenter, angle, wheels::LEFT);
        double rightDistance = calculateArcWheelDistances(arcCenter, angle, wheels::RIGHT);

        if(mode == speedMode::CONTROLLED)
        {
            trj.makePositionBased(velocity, tachoToCm(max(abs(leftDistance), abs(rightDistance))), linearAcceleration, startLinearVelocity, endLinearVelocity);
            double empty;
            Kp = KpArc;
            // velocity = cmToTacho(velocity);
            while(abs(getAngle()) < angle)
            {
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                if(velocity == 0) break;

                wheelSpeeds = calculateArcSpeeds(cmToTacho(velocity), leftDistance, rightDistance, arcCenter, angle);           
                actuateControlled(wheelSpeeds.left, wheelSpeeds.right);
            }
        }
        else
        {
            wheelSpeeds = calculateArcSpeeds(cmToTacho(velocity), leftDistance, rightDistance, arcCenter, angle);

            // while((abs(leftMotor->getTachoCount()) + abs(rightMotor->getTachoCount())) / 2 < (leftDistance + rightDistance) / 2)
            while(abs(getAngle()) < angle)
            {
                actuateMotors(wheelSpeeds.left, wheelSpeeds.right);
            }
        }

        stop(stopMode);
    }

    //Time Based
    void chassis::tankTime(int leftSpeed, int rightSpeed, double seconds, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        t.reset();

        while(t.secElapsed() < seconds)
        {
            actuateMotors(leftSpeed, rightSpeed);
        }

        stop(stopMode);
    }

    void chassis::straightTime(double velocity, double seconds, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        t.reset();

        trj.makeTimeBased(velocity, seconds, linearAcceleration, startLinearVelocity, endLinearVelocity);

        double empty;
        Kp = KpRegular;
        while(t.secElapsed() < seconds)
        {
            trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
            velocity = cmToTacho(velocity);
            actuateControlled(velocity, velocity);
        }

        stop(stopMode);
    }

    void chassis::turnTime(double angularVelocity, double seconds, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();
    
        t.reset();
        distanceController.startHold(0);
        headingController.startTime(seconds, angularVelocity, angularAcceleration, TARGET_TIME, startAngularVelocity, endAngularVelocity);

        Kp = KpRegular;
        while(t.secElapsed() < seconds)
        {
            actuateControlledExternal(t.secElapsed());
        }

        stop(stopMode);
    }

    void chassis::arcTime(double velocity, double seconds, double arcCenter, direction dir, breakMode stopMode)
    {
        setMode(mode);
        resetPosition();

        velocity = clamp(velocity, -getTachoSpeedLimit(), getTachoSpeedLimit());

        t.reset();

        double leftDistance, rightDistance, angle;

        if(dir == FORWARD) 
        {
            angle = 1; //POSITIVE DIRECTION
            leftDistance = calculateArcWheelDistances(arcCenter, angle, wheels::LEFT);
            rightDistance = calculateArcWheelDistances(arcCenter, angle, wheels::RIGHT);
        }
        else 
        {
            angle = -1; //NEGATIVE DIRECTION
            leftDistance = calculateArcWheelDistances(arcCenter, angle, wheels::LEFT);
            rightDistance = calculateArcWheelDistances(arcCenter, angle, wheels::RIGHT);
        }

        if(mode == speedMode::CONTROLLED)
        {
            trj.makeTimeBased(velocity, seconds, linearAcceleration, startLinearVelocity, endLinearVelocity);
            double empty;
            Kp = KpArc;
            while(t.secElapsed() < seconds)
            {
                trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
                velocity = cmToTacho(velocity);
                wheelSpeeds = calculateArcSpeeds(velocity, leftDistance, rightDistance, arcCenter, angle);
                actuateControlled(wheelSpeeds.left, wheelSpeeds.right);
            }
        }
        else
        {
            wheelSpeeds = calculateArcSpeeds(velocity, leftDistance, rightDistance, arcCenter, angle);

            while(t.secElapsed() < seconds)
            {
                actuateMotors(wheelSpeeds.left, wheelSpeeds.right);
            }
        }

        stop(stopMode);
    }

    //Unlimited
    void chassis::tankUnlim(int leftSpeed, int rightSpeed, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetPosition();

        actuateMotors(leftSpeed, rightSpeed);
    }

    void chassis::straightUnlim(double velocity, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho)
        {
            trj.makeTimeBased(velocity, DURATION_FOREVER, linearAcceleration, startLinearVelocity, endLinearVelocity);
            t.reset();
            resetPosition();
            Kp = KpRegular;
        }
        double empty;
        trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
        velocity = cmToTacho(velocity);
        actuateControlled(velocity, velocity);
    }

    void chassis::turnUnlim(double angularVelocity, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho)
        {
            distanceController.startHold(0);
            headingController.startTime(DURATION_FOREVER, angularVelocity, angularAcceleration, checkTargetType::TARGET_NEVER, startAngularVelocity, endAngularVelocity);
            t.reset();
            resetPosition();
            Kp = KpRegular;
        }
        actuateControlledExternal(t.secElapsed());
    }

    void chassis::arcUnlim(double velocity, double arcCenter, direction dir, bool resetTacho)
    {
        setMode(mode);
        velocity = clamp(velocity, -getTachoSpeedLimit(), getTachoSpeedLimit());
        double leftDistance, rightDistance, angle;
        if(dir == FORWARD) 
        {
            angle = 1; //POSITIVE DIRECTION
            leftDistance = calculateArcWheelDistances(arcCenter, angle, wheels::LEFT);
            rightDistance = calculateArcWheelDistances(arcCenter, angle, wheels::RIGHT);
        }
        else 
        {
            angle = -1; //NEGATIVE DIRECTION
            leftDistance = calculateArcWheelDistances(arcCenter, angle, wheels::LEFT);
            rightDistance = calculateArcWheelDistances(arcCenter, angle, wheels::RIGHT);
        }

        if(resetTacho)
        {
            resetPosition();
            t.reset();
            trj.makeTimeBased(velocity, DURATION_FOREVER, linearAcceleration, startAngularVelocity, endAngularVelocity);
            Kp = KpArc;
        }
        double empty;
        if(mode == speedMode::CONTROLLED)
        {
            // trj.getReference(t.secElapsed(), &empty, &velocity, &empty);
            velocity = cmToTacho(velocity);
            wheelSpeeds = calculateArcSpeeds(velocity, leftDistance, rightDistance, arcCenter, angle);
            actuateControlled(wheelSpeeds.left, wheelSpeeds.right);
        }
        else
        {
            wheelSpeeds = calculateArcSpeeds(cmToTacho(velocity), leftDistance, rightDistance, arcCenter, angle);
            actuateMotors(wheelSpeeds.left, wheelSpeeds.right);
        }
    }

    //Until Stalled
    void chassis::tankStalled(int leftSpeed, int rightSpeed, double maxTimeLimit, breakMode stopMode, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetPosition();

        t.reset();
        double speedTolerance = mode == UNREGULATED ? speedTolerancePCT : speedToleranceDPS;
        tankUnlim(leftSpeed, rightSpeed);
        while(t.secElapsed() < maxTimeLimit)
        {
            if(abs((abs(leftSpeed) + abs(rightSpeed)) / 2 - (abs(leftMotor->getCurrentSpeed()) + abs(rightMotor->getCurrentSpeed())) / 2) > speedTolerance)
            {
                t.secDelay(stallTime);
                if(abs((abs(leftSpeed) + abs(rightSpeed)) / 2 - (abs(leftMotor->getCurrentSpeed()) + abs(rightMotor->getCurrentSpeed())) / 2) > speedTolerance)break;
            }
        }

        stop(stopMode);
    }

    void chassis::straightStalled(double velocity, double maxTimeLimit, breakMode stopMode, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetPosition();
        
        t.reset();
        timer stallTimer;
        straightUnlim(velocity, true);
        while(t.secElapsed() < maxTimeLimit)
        {
            if(abs(abs(velocity) - abs(getLinearVelocity())) > speedToleranceLinear)
            {
                stallTimer.reset();
                while(stallTimer.secElapsed() < stallTime)
                {
                    straightUnlim(velocity);
                }
                if(abs(abs(velocity) - abs(getLinearVelocity())) > speedToleranceLinear)break;
            }
            straightUnlim(velocity);
        }

        stop(stopMode);
    }

    void chassis::turnStalled(double angularVelocity, double maxTimeLimit, breakMode stopMode, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetPosition();
        
        t.reset();
        timer stallTimer;
        turnUnlim(angularVelocity, true);
        while(t.secElapsed() < maxTimeLimit)
        {
            if(abs(abs(angularVelocity) - abs(getAngularVelocity())) > speedToleranceAngular)
            {
                stallTimer.reset();
                while(stallTimer.secElapsed() < stallTime)
                {
                    turnUnlim(angularVelocity);
                }
                if(abs(abs(angularVelocity) - abs(getAngularVelocity())) > speedToleranceAngular)break;
            }
            turnUnlim(angularVelocity);
        }

        stop(stopMode);
    }

    void chassis::arcStalled(double velocity, double arcCenter, direction dir, double maxTimeLimit, breakMode stopMode, bool resetTacho)
    {
        setMode(mode);
        if(resetTacho) resetPosition();
        t.reset();
        
        double robotVelocity = velocity * (abs(arcCenter) / (abs(arcCenter) + chassisRadius));

        if(mode == speedMode::CONTROLLED)
        {
            timer stallTimer;
            arcUnlim(velocity, arcCenter, dir, true);
            while(t.secElapsed() < maxTimeLimit)
            {
                if(abs(abs(robotVelocity) - abs(getLinearVelocity())) > speedToleranceLinear)
                {
                    stallTimer.reset();
                    while(stallTimer.secElapsed() < stallTime)
                    {
                        arcUnlim(velocity, arcCenter, dir);
                    }
                    if(abs(abs(robotVelocity) - abs(getLinearVelocity())) > speedToleranceLinear)break;
                }
                arcUnlim(velocity, arcCenter, dir);
            }
        }
        else
        {
            double speedTolerance = mode == UNREGULATED ? speedTolerancePCT : speedToleranceDPS;
            arcUnlim(velocity, arcCenter, dir, resetTacho);
            while(t.secElapsed() < maxTimeLimit)
            {
                if(abs(abs(robotVelocity) - (abs(leftMotor->getCurrentSpeed()) + abs(rightMotor->getCurrentSpeed())) / 2) > speedTolerance)
                {
                    t.secDelay(stallTime);
                    if(abs(abs(robotVelocity) - (abs(leftMotor->getCurrentSpeed()) + abs(rightMotor->getCurrentSpeed())) / 2) > speedTolerance)break;
                }
            }
        }
        

        stop(stopMode);
    }
}
