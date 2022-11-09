#ifndef _8059_SIMPLE_PID_CONTROL_HPP_
#define _8059_SIMPLE_PID_CONTROL_HPP_
extern double errorEncdL, errorEncdR, targPowerL, targPowerR, powerL, powerR, errorBearing;
extern bool turnMode;
void setMaxPow(double pow);
void enableBase(bool left, bool right);
void baseMove(double dis, double kp, double kd);
void baseMove(double dis);
void baseTurn(double bearing, double kp, double kd);
void baseTurn(double bearing);
void powerBase(double l, double r);
void timerBase(double l, double r, double t);
void unPauseBase();
void waitBase(double cutoff);
void Control(void * ignore);
void resetCoords(double x, double y);
#endif
