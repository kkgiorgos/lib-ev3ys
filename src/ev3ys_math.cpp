#include "ev3ys_math.h"
#include <algorithm>

using namespace std;

int sign(double value)
{
    if(value > 0) return 1;
    if(value < 0) return -1;
    return 0;
}

double clamp(double value, double minimum, double maximum)
{
    return min(maximum, max(minimum, value));
}

double absolute(double value)
{
    if(value >= 0) return value;
    else return -value;
}

double radToDeg(double rads)
{
    return (rads * 180) / MATH_PI;
}

double degToRad(double degs)
{
    return degs * (MATH_PI / 180);
}

bool checkRange(double value, double median, double range)
{
    range /= 2;
    return (median + range) > value && (median - range) < value;
}
