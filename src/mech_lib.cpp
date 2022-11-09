#include "main.h"

const double catSpeed = 127, catKP = 0.5, catMaxHeight = 7400, minUpHeight = 35000;
const double intakeSpeed = 127;

bool shootState = false, catDown = false, catClear = true;
void catControl(void*ignore) {
    Motor cat(catPort);
    Rotation rot(rotPort);

    while (true) {
        double error = rot.get_angle() > minUpHeight ? catMaxHeight : catMaxHeight - rot.get_angle();
        
        if(shootState) {
            cat.move(catSpeed);
            delay(300);
            
            cat.move(0);
            shootState = false;
            catClear = false;
            // shootState = error < 0;
        }else if(!catClear) {
            catClear = error > 7000;
        }else {
            cat.move(error*catKP);
        }

        catDown = rot.get_angle() > catMaxHeight/2;

        printf("rot: %d, shootState: %d, catdown: %d\n", rot.get_angle(), shootState, catDown);
        delay(5);
    }
}

void shootCat() {shootState = true;}

double targIntakeSpeed = 0;
bool flip = false;
void intakeControl(void*ignore){
    Motor intake(intakePort);

    while(true) {
        if(catDown) {
            if(flip) {
                intake.move(targIntakeSpeed);
                delay(200);
                intake.move(0);
                flip = false;
            }else {
                intake.move(targIntakeSpeed);
                // printf("targsped: %.2f\n", targIntakeSpeed);
                delay(5);
            }
        }else {
            intake.move(0);
        }   
        delay(5);
    }
}

void setIntakeTarget(double s) {targIntakeSpeed = s;}
void flipRoller() {flip = true;}