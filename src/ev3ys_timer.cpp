#include "ev3ys_timer.h"

using namespace ev3cxx;

namespace ev3ys
{
    timer::timer()
    {
        reset();
    }

    void timer::reset()
    {
        startElapsed = s.getUs();
    }

    double timer::secElapsed()
    {
        return (s.getUs() - startElapsed) / 1000000.0;
    }

    void timer::secDelay(double seconds)
    {
        wait(seconds * 1000);
    }
}
