#pragma once

#include "ev3cxx.h"
#include "ev3ys_motor.h"
#include "ev3ys_math.h"

namespace ev3ys
{
    enum direction
    {
        FORWARD,
        BACKWARD
    };

    enum wheels
    {
        LEFT = -1,
        RIGHT = 1
    };

    struct speeds
    {
        double left;
        double right;
    };

    class chassis
    {
    private:
        motor *leftMotor;
        motor *rightMotor;
        speedMode mode;
        timer t;

        control distanceController;
        control headingController;

        trajectory trj;
        ev3cxx::Bluetooth bt;

        double startLinearVelocity, endLinearVelocity;
        double startAngularVelocity, endAngularVelocity;
        double linearAcceleration;
        double angularAcceleration;

        double stallTime;
        int speedTolerancePCT;
        int speedToleranceDPS;
        double speedToleranceLinear;
        double speedToleranceAngular;
        double unregulatedDPS;

        double wheelCircumference;
        double chassisRadius;
        double axleLength;
        speeds wheelSpeeds{};

        double Kp, Kd, KpRegular, KpArc;
        double lastError;

        double tachoToCm(double tacho);
        double angularToTacho(double angular);

        

        speeds calculateArcSpeeds(double velocity, double leftDistance, double rightDistance, double center, double angle);

        void actuateMotors(double leftSpeed, double rightSpeed);
        bool actuateControlledExternal(double time);

    public:
        chassis(motor *leftMotor, motor *rightMotor, double wheelDiameter, double axleLength, double KpRegular, double KpArc, double Kd);

        void setMode(speedMode mode);
        speedMode getMode();
        int getTachoSpeedLimit();
        void setLinearAccelParams(double acceleration, double startSpeed, double endSpeed);
        void setAngularAccelParams(double acceleration, double startSpeed, double endSpeed);
        void setStallTolerance(int speedTolerancePCT, int speedToleranceDPS, double speedToleranceLinear, double speedToleranceAngular, double stallTime);
        void setUnregulatedDPS(bool isDPS = true);


        double cmToTacho(double cm);
        double getLinearVelocity();
        double getAngularVelocity();
        double getPosition();
        double getAngle();
        double getKp();
        bool getUnregulatedDPS();
        void resetPosition();

        void stop(breakMode stopMode = breakMode::COAST);

        void actuateKinematically(double linearVelocity, double angularVelocity);
        void actuateControlled(double leftSpeed, double rightSpeed);
        double calculateArcWheelDistances(double center, double angle, wheels wheel);
        double calculateLinear(double leftTacho, double rightTacho);
        double calculateAngular(double leftTacho, double rightTacho);

        //Straight and Turn only controlled tank only unregulated/regulated Arc is for everything

        //Velocity at arcs is linearVelocity of the fastest wheel

        //unlimited CONTROLLED modes have to be repeatedly called.  To start new maneuver resetTacho otherwise it continues the previous

        //Position Based
        void tank(int leftSpeed, int rightSpeed, int degrees, breakMode stopMode = breakMode::BRAKE);
        void straight(double velocity, double distance, breakMode stopMode = breakMode::BRAKE);
        void turn(double angularVelocity, double angle, breakMode stopMode = breakMode::BRAKE);
        void arc(double velocity, double angle, double arcCenter, breakMode stopMode = breakMode::BRAKE);

        //Time Based
        void tankTime(int leftSpeed, int rightSpeed, double seconds, breakMode stopMode = breakMode::BRAKE);
        void straightTime(double velocity, double seconds, breakMode stopMode = breakMode::BRAKE);
        void turnTime(double angularVelocity, double seconds, breakMode stopMode = breakMode::BRAKE);
        void arcTime(double velocity, double seconds, double arcCenter, direction dir, breakMode stopMode = breakMode::BRAKE);

        //Unlimited
        void tankUnlim(int leftSpeed, int rightSpeed, bool resetTacho = false);
        void straightUnlim(double velocity, bool resetTacho = false);
        void turnUnlim(double angularVelocity, bool resetTacho = false);
        void arcUnlim(double velocity, double arcCenter, direction dir, bool resetTacho = false);

        //Until stalled
        void tankStalled(int leftSpeed, int rightSpeed, double maxTimeLimit = 1.0, breakMode stopMode = breakMode::HOLD, bool resetTacho = false);
        void straightStalled(double velocity, double maxTimeLimit = 1.0, breakMode stopMode = breakMode::HOLD, bool resetTacho = false);
        void turnStalled(double angularVelocity, double maxTimeLimit = 1.0, breakMode stopMode = breakMode::HOLD, bool resetTacho = false);
        void arcStalled(double velocity, double arcCenter, direction dir, double maxTimeLimit = 1.0, breakMode stopMode = breakMode::HOLD, bool resetTacho = false);
    };
}
