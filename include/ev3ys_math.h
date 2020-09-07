#ifndef EXTRAMATH_H_INCLUDED
#define EXTRAMATH_H_INCLUDED

#define MATH_PI (3.14)

int sign(double value); //Returns the sign of a given number
double clamp(double value, double minimum, double maximum);
double absolute(double value);
double radToDeg(double rads);
double degToRad(double degs);
bool checkRange(double value, double median, double range);

#endif // EXTRAMATH_H_INCLUDED
