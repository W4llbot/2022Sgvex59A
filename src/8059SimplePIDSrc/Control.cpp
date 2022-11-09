#include "globals.hpp"
#include "main.h"
#define DEFAULT_KP 0.2
/*  10 = 0.35,
    15 = 0.3,
    20-30-35 = 0.27,
    40-50-60 = 0.25 */
#define DEFAULT_KD 0.1
#define DEFAULT_TURN_KP 5
/*  20 = 2.5,
    30 = 2.1,
    40 = 1.8,
    50 = 1.6,
    60 = 1.5,
    70 = 1.4,
    80 = 1.35,
    90 = 1.3,
    100 = 1.25,
    110 = 1.2,
    120 = 1.15,
    130 = 1.13,
    140 = 1.11,
    150 = 1.08,
    160 = 1.05,
    170 = 1.02,
    175 = 0.98,
    180 = 0.967 */
#define DEFAULT_TURN_KD 30
#define RAMPING_POW 1000
#define DISTANCE_LEEWAY 15
#define BEARING_LEEWAY 1.5
double MAX_POW = 80;

double targEncdL = 0, targEncdR = 0, targBearing = 0;
double errorEncdL = 0, errorEncdR = 0, errorBearing = 0;
double powerL = 0, powerR = 0;
double targPowerL = 0, targPowerR = 0;
double kP = DEFAULT_KP, kD = DEFAULT_KD;

bool turnMode = false, pauseBase = false;
bool enableLeft = true, enableRight = true;

void setMaxPow(double pow) {
  MAX_POW = pow;
}

void enableBase(bool left, bool right) {
  enableLeft = left;
  enableRight = right;
}

void baseMove(double dis, double kp, double kd){
  printf("baseMove(%.2f)\n", dis);
  turnMode = false;
  targEncdL += dis/inPerDeg;
  targEncdR += dis/inPerDeg;

  kP = kp;
  kD = kd;
}
void baseMove(double dis){
  baseMove(dis, DEFAULT_KP, DEFAULT_KD);
}

void baseTurn(double p_bearing, double kp, double kd){
  printf("baseTurn(%.2f, %.2f, %.2f)\n", p_bearing, kp, kd);
  turnMode = true;
  targBearing = p_bearing;
	kP = kp;
	kD = kd;
}
void baseTurn(double bearing){
  baseTurn(bearing, DEFAULT_TURN_KP, DEFAULT_TURN_KD);
}

void powerBase(double l, double r) {
  printf("powerBase(%.2f, %.2f)\n", l, r);
  pauseBase = true;
  powerL = l;
  powerR = r;
}

void timerBase(double l, double r, double t) {
  printf("timerBase(%.2f, %.2f, %.2f)\n", l, r, t);
  pauseBase = true;
  powerL = l;
  powerR = r;
  delay(t);
  powerL = 0;
  powerR = 0;
  pauseBase = false;
  resetCoords(X, Y);
}

void unPauseBase() {
  powerL = 0;
  powerR = 0;
  pauseBase = false;
  resetCoords(X, Y);
}

void waitBase(double cutoff){
	double start = millis();
  if(turnMode) {
    while(fabs(targBearing - bearing) > BEARING_LEEWAY && (millis()-start) < cutoff) delay(20);
  }else{
    while((fabs(targEncdL - encdL) > DISTANCE_LEEWAY || fabs(targEncdR - encdR) > DISTANCE_LEEWAY) && (millis()-start) < cutoff) delay(20);
  }

  targEncdL = encdL;
  targEncdR = encdR;
  printf("Tima taken %.2f\n", (millis() - start));
  delay(200);
}

void Control(void * ignore){
  Motor FL (FLPort);
  Motor ML (MLPort);
  Motor BL (BLPort);
  Motor FR (FRPort);
  Motor MR (MRPort);
  Motor BR (BRPort);
  Imu imu (imuPort);

  double prevErrorEncdL = 0, prevErrorEncdR = 0, prevErrorBearing = 0;
  while(competition::is_autonomous()){
    // printf("running\n");
    if(!imu.is_calibrating() && !pauseBase) {
      if(turnMode){
        errorBearing = targBearing - bearing;
        double deltaErrorBearing = errorBearing - prevErrorBearing;

        if(enableLeft&&enableRight) {
          targPowerL = errorBearing * kP + deltaErrorBearing * kD;
          targPowerR = -targPowerL;
        }else {
          targPowerL = enableLeft ? errorBearing * kP + deltaErrorBearing * kD : 0;
          targPowerR = enableRight ? -errorBearing * kP + deltaErrorBearing * kD : 0;
        }
        

        prevErrorBearing = errorBearing;
      }else{
        errorEncdL = targEncdL - encdL;
        errorEncdR = targEncdR - encdR;

        double error = (errorEncdL+errorEncdR)/2;

        double deltaErrorEncdL = error - prevErrorEncdL;
        double deltaErrorEncdR = error - prevErrorEncdR;

        targPowerL = error * kP + deltaErrorEncdL * kD;
        targPowerR = error * kP + deltaErrorEncdR * kD;

        prevErrorEncdL = error;
        prevErrorEncdR = error;
      }

      double deltaPowerL = targPowerL - powerL;
      powerL += abscap(deltaPowerL, RAMPING_POW);
      double deltaPowerR = targPowerR - powerR;
      powerR += abscap(deltaPowerR, RAMPING_POW);

      powerL = abscap(powerL, MAX_POW);
      powerR = abscap(powerR, MAX_POW);
    }
    FL.move(powerL);
    ML.move(powerL);
    BL.move(powerL);
    FR.move(powerR);
    MR.move(powerR);
    BR.move(powerR);
    delay(5);
  }
}

void resetCoords(double x, double y){
  Motor FL (FLPort);
  Motor ML (MLPort);
  Motor BL (BLPort);
  Motor FR (FRPort);
  Motor MR (MRPort);
  Motor BR (BRPort);

  FL.tare_position();
  ML.tare_position();
  FR.tare_position();
  BL.tare_position();
  MR.tare_position();
  BR.tare_position();
  resetPrevEncd();

  targBearing = bearing;
  targEncdL = 0;
  targEncdR = 0;

  setCoords(x, y);
}
