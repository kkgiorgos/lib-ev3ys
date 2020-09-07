#pragma once

#define DURATION_FOREVER (-1)

namespace ev3ys
{
    class trajectory
    {
    private:
        bool isForever;
        //Times in seconds
        double t0; //Start time
        double t1; //Time at the end of acceleration phase
        double t2; //Time at the start of deceleration phase
        double t3; //Time at the end of the trajectory
        //Positions
        double p0; //Start position
        double p1; //Position at the end of acceleration phase
        double p2; //Position at the start of deceleration phase
        double p3; //Position at the end of the trajectory
        //Velocities
        double v0; //Velocity at the start
        double v1; //Velocity at constant phase
        double v_max; //Max Possible Velocity
        //Accelerations
        double a; //Acceleration at the acceleration phase
        double d; //Acceleration at the deceleration phase
        double a_max;//Max Possible Acceleration

        void reverseTrajectory(); //Changes trajectory when backwards

        double calcVelocity(double a, double dt); //Calculates Velocity with time-based equations
        double calcPosition(double a, double dt); //Calculates Position with time-based equations when accelerating/decelerating
        double calcPositionConstant(double v, double dt); //Calculated Position with constant velocity with time-based equations
        double calcTime(double dv, double a); //Calculated duration of velocity change with specific acceleration
    public:
        trajectory();
        void setLimits(double maxVelocity, double maxAcceleration);

        double getStartTime();
        double getEndTime();
        double getStartPos();
        double getEndPos();

        void makeStationary(double startTime, double startPosition);
        void makeTimeBased(double targetVelocity, double duration, double acceleration, double startVelocity, double endVelocity = 0, double startPosition = 0, double startTime = 0);
        void makePositionBased(double targetVelocity, double distance, double acceleration, double startVelocity, double endVelocity = 0, double startPosition = 0, double startTime = 0);

        void getReference(double time, double *position, double *velocity, double *acceleration);
    };
}
