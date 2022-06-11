#pragma once

#include "ev3cxx.h"

namespace ev3ys
{
    class timer
    {
    private:
        ev3cxx::StopWatch s;

        int startElapsed;	//Starting time point for Elapsed function

    public:
        timer();

        void reset();			//Reset timer (Set new Starting time point for Elapsed function)
        double secElapsed();	//Measures time difference between now and starting time point

        static void secDelay(double seconds);	//Runs a while loop for inputed seconds using the same method as secElapsed()
    };
}
