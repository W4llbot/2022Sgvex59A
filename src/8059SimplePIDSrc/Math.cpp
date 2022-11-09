#include "main.h"
double boundRad(double rad){
  double res = fmod(rad, twoPI);
  if(res < 0) res += twoPI;
  return res;
}
double boundDeg(double deg){
  double res = fmod(deg, 360);
  if(res < 0) res += 360;
  return res;
}
double abscap(double x, double abscap){
  if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}
