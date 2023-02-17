#pragma once

#include "ev3cxx.h"
#include "ev3ys_timer.h"
#include "ev3ys_colorSensor.h"
#include "ev3ys_chassis.h"
#include "ev3ys_math.h"
#include "ev3ys_motor_trajectory.h"
#include <map>

namespace ev3ys
{
    enum sensorModes
    {
        REFLECTED,
        WHITE_RGB
    };

    enum followModes
    {
        TWO_SENSOR,
        LEFT_SENSOR_LEFT_POSITION,
        LEFT_SENSOR_RIGHT_POSITION,
        RIGHT_SENSOR_LEFT_POSITION,
        RIGHT_SENSOR_RIGHT_POSITION,
        ONE_SENSOR_LEFT_POSITION,
        ONE_SENSOR_RIGHT_POSITION,
        NO_SENSORS
    };

    struct PID_params
    {
        double Kp;
        double Ki;
        double Kd;
    };

    class lineFollower
    {
    private:
        chassis *driveBase;
        colorSensor *sensor;
        colorSensor *leftSensor;
        colorSensor *rightSensor;
        timer periodTimer;
        timer t;
        trajectory trj;
        ev3cxx::Bluetooth bt;

        speedMode chassisMode;
        bool chassisUnregulatedDPS;
        sensorModes sensorMode;
        int sensorAmount;
        speedMode motionMode;
        followModes followMode;

        double acceleration, startSpeed, endSpeed;

        double loopPeriod;
        double kp, ki, kd;
        std::map<double, PID_params> pidSpeedParams; 
        bool forcedParams;
        bool lineDetected;

        double scaling;

        double integral, lastError;
        double target;

        void setSpeedMode();
        void resetChassisMode();

        void resetPID(double velocity);
        double calculateError();

        void runPID(double speed);

    public:
        lineFollower(int loopFrequency, chassis *driveBase, colorSensor *sensor);
        lineFollower(int loopFrequency, chassis *driveBase, colorSensor *leftSensor, colorSensor *rightSensor);

        void setAccelParams(double acceleration, double startSpeed, double endSpeed);
        void setSensorMode(sensorModes mode);
        void initializeMotionMode(speedMode mode);
        void addPIDparams(double velocity, double kp, double ki, double kd);
        void setPIDparams(double kp, double ki, double kd);
        void forcePIDparams(bool doForce = true);

        void setDoubleFollowMode(const char *leftPos, const char *rightPos);
        void setSingleFollowMode(const char *leftPos, const char *rightPos);

        bool getLineDetected() {return lineDetected;}

        void stop(breakMode stopMode = breakMode::COAST);

        //Velocity units: PCT -> UNREGULATED, DPS -> REGULATED, CMPS -> CONTROLLED
        //Distance units: CM
        void distance(double velocity, double distance, breakMode stopMode = breakMode::BRAKE);
        void seconds(double velocity, double seconds, breakMode stopMode = breakMode::BRAKE);
        void unlimited(double velocity, bool reset = false);
        void lines(double velocity, int lines = 1, breakMode stopMode = breakMode::BRAKE, double distanceBeforeLine = 5, double lineSkipDistance = 2, bool addFinalLineSkip = false);
    };
}
