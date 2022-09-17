#pragma once

#include "ev3cxx.h"
#include "ev3ys_timer.h"
#include "ev3ys_motor_controller.h"
#include "ev3ys_math.h"


namespace ev3ys
{
    enum speedMode
    {
        UNREGULATED,
        REGULATED,
        CONTROLLED
    };

    enum motorType
    {
        LARGE,
        MEDIUM
    };

    enum breakMode
    {
        HOLD,
        BRAKE,
        COAST,
        BRAKE_COAST,
        ZERO,
        NONE
    };

    class motor: protected ev3cxx::Motor
    {
    private:
        static const int maxMediumSpeed;
        static const int maxLargeSpeed;
        static const int maxUnregSpeedPCT;
        static const int maxUnregSpeedDPS;
        static const int speedLimit;

        int direction;

        speedMode mode;
        motorType type;
        int maxSpeed;
        int speedToleranceDPS;
        int speedTolerancePCT;
        double stallTime;
        bool unregulatedDPS;    //True: use DPS as unreg unit, False: use the usual PCT(%) units
        bool limitSpeed;
        bool stallTimerStarted;

        int tachoCountReset;

        double acceleration, startSpeed, endSpeed;

        control motorController;
        timer t;

        int trimSpeed(int speed);

    public:
        motor(ev3cxx::MotorPort port, bool inversed, ev3cxx::MotorType type = ev3cxx::MotorType::MEDIUM); //ALLOWED TYPES FOR MOTORS ONLY LARGE AND MEDIUM
        
        void setMode(speedMode mode);
        speedMode getMode();
        void setAccelParams(int acceleration, int startSpeed = 0, int endSpeed = 0);
        void setStallTolerance(int speedToleranceDPS, int speedTolerancePCT, double stallTime);
        void setUnregulatedDPS(bool isDPS = true);
        void setSpeedLimiter(bool doLimit);

        int getSpeedLimit();

        int dps_to_pct(int speed);  //Converts speed from degrees per second(dps) to % of maximum
        int pct_to_dps(int speed);  //Converts speed from % of maximum to degrees per second(dps)

        int getCurrentSpeed();
        int getTachoCount();
        int getAbsoluteTachoCount();
        void resetTachoCount();
        void initialiseAbsoluteCount();

        bool isStalled(int speed);

        void stop(breakMode stopMode = breakMode::COAST);
        void moveUnlimited(int speed, bool resetTacho = false);
        void moveDegrees(int speed, int degrees, breakMode stopMode = breakMode::COAST, bool wait = true);
        void moveDegreesAbsolute(int speed, int degrees, breakMode stopMode = breakMode::HOLD);
        void moveSeconds(int speed, double seconds, breakMode stopMode = breakMode::COAST);
        void moveUntilStalled(int speed, breakMode stopMode = breakMode::HOLD, double maxTimeLimit = 1, bool resetTacho = false);
    };
}
