#ifndef _8059_SIMPLE_PID_MATH_HPP_
#define _8059_SIMPLE_PID_MATH_HPP_
#define PI      3.14159265358979323846264338328
#define halfPI  1.57079632679489661923132169164
#define twoPI	  6.28318530717958647692528676656
/**
 * angle conversion from radians to degrees and vice versa
 * angleDeg = angle * toDeg
 * angle = angleDeg * toRad
 */
#define toDeg   57.2957795130823208767981548141
#define toRad   0.0174532925199432957692369076849
/** refer to mathUtils.cpp for function documentation */
double boundRad(double rad);
double boundDeg(double deg);
double abscap(double x, double abscap);
#endif
