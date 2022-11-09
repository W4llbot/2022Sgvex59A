#include "main.h"
#include "8059SimplePIDInclude/Control.hpp"
#include "globals.hpp"
#include "mech_lib.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor cat(catPort, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
	Motor intake(intakePort, E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
	Rotation rot(rotPort);

	Motor FL(FLPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor ML(MLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor MR(MLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	FL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	ML.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	MR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Task catControlTask(catControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Cat Control Task");
	Task intakeControlTask(intakeControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Control Task");
	Task sensorTask(Sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Sensors Task");
	Task debugTask(Debug, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");

	Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(Control, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void roller() {
	Motor FL(FLPort);
	Motor ML(MLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor MR(MRPort);
	Motor BR(BRPort);
	
	powerBase(-100, -100);
	delay(300);
	powerBase(100, 100);
	delay(200);
	powerBase(0, 0);
	delay(300);
	powerBase(-100, -100);
	delay(300);
	powerBase(0, 0);
}
// goal -12, 111
// shooting pt 45, 54
void autonomous() {
	double start = millis();

	// roller();
	//delay(300);
	resetCoords(0, 0);
	// baseMove(24, .2, 0.1);
	// waitBase(9999999);
	// enableBase(true, false);
	// baseTurn(90, 5, 30);

	// baseTurn(90, 5, 30);
	// waitBase(9999);



	// resetPP();
	// resetCoord();
	// setMaxRPMV(400);
	// waitPP(0);
	roller();
	// delay(100);
	setIntakeTarget(127);
	unPauseBase();
	enableBase(true, false);
	baseTurn(40);
	waitBase(2000);
	// delay(500);
	enableBase(true, true);
	baseMove(67);
	waitBase(2500);
	baseTurn(-43);
	waitBase(1000);
	// baseMove(4);
	// waitBase(1000);
	delay(200);
	shootCat();
	delay(500);
	// baseMove(-4);
	// waitBase(1000);
	enableBase(true, false);
	baseTurn(43.4-180);
	waitBase(2000);
	// delay(500);
	enableBase(true, true);
	
	setMaxPow(70);
	baseMove(-68);
	setIntakeTarget(127);
	waitBase(2500);
	setMaxPow(80);
	baseTurn(-90);
	waitBase(1000);
	setIntakeTarget(0);

	roller();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	Motor FL(FLPort);
	Motor ML(MLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor MR(MRPort);
	Motor BR(BRPort);
	ADIDigitalOut piston1(piston1Port, LOW);
	ADIDigitalOut piston2(piston2Port, LOW);

	FL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	ML.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	MR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	bool tankDrive = true;
	
	while(true){
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;
		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		} else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);
			left = power + turn;
			right = power - turn;
		}

		FL.move(left);
		ML.move(left);
		BL.move(left);
		FR.move(right);
		MR.move(right);
		BR.move(right);

		if(master.get_digital(DIGITAL_R1)) shootCat();
		if(master.get_digital(DIGITAL_R2)) {
			piston1.set_value(true);
			piston2.set_value(true);
		}
		if(master.get_digital(DIGITAL_X)) roller();
		
		setIntakeTarget((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2)) * 127);

		delay(5);
	}
 }	
