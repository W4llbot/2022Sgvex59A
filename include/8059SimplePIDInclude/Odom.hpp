#ifndef _8059_SIMPLE_PID_ODOM_HPP_
#define _8059_SIMPLE_PID_ODOM_HPP_
extern double X, Y;
extern const double inPerDeg;
void setCoords(double x, double y);
void Odometry(void * ignore);
void resetPrevEncd();
#endif
