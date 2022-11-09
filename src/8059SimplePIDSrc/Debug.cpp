#include "main.h"
int DEBUG_MODE = -1;
void printPosMaster(){
  Controller master(E_CONTROLLER_MASTER);
  Imu imu (imuPort);
  if(imu.is_calibrating()) master.print(2, 0, "Callibrate IMU");
  else master.print(2, 0, "%.2f %.2f %.2f", X, Y, bearing);
}
void printPosTerminal(){
  printf("x: %.2f y: %.2f bearing: %.2f\n", X, Y, bearing);
}
void printEncdTerminal(){
  printf("encdL: %.2f encdR: %.2f\n", encdL, encdR);
}
void printErrorTerminal(){
  if(turnMode) printf("errorBearing: %.2f\n", errorBearing);
  else printf("errorEncdL: %.2f errorEncdR: %.2f\n", errorEncdL, errorEncdR);
}
void printTargPowerTerminal(){
  printf("targPowerL: %.2f, targPowerR: %.2f\n", targPowerL, targPowerR);
}
void printPowerTerminal(){
  printf("powerL: %.2f powerR: %.2f\n", powerL, powerR);
}
void Debug(void * ignore){
  Imu imu (imuPort);
  while(true){
    printPosMaster();
    if(imu.is_calibrating()) {
      printf("imu is calibrating...\n");
    }else {
      switch(DEBUG_MODE){
        case 1: printPosTerminal(); break;
        case 2: printEncdTerminal(); break;
        case 3: printErrorTerminal(); break;
        case 4: printTargPowerTerminal(); break;
        case 5: printPowerTerminal(); break;
      }
    }
    delay(50);
  }
}
