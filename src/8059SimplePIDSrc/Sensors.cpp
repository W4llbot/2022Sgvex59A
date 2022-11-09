#include "main.h"
double encdL = 0, encdR = 0, bearing = 0, angle = halfPI;

void Sensors(void * ignore){
  Motor FL (FLPort);
  Motor ML (MLPort);
  Motor BL (BLPort);
  Motor FR (FRPort);
  Motor MR (MRPort);
  Motor BR (BRPort);
  Imu imu (imuPort);
  while(true){
    if(!imu.is_calibrating()){
      encdL = ML.get_position();
      encdR = MR.get_position();
      bearing = imu.get_rotation();
      angle = halfPI - bearing * toRad;
    }
    delay(5);
  }
}
