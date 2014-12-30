#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     irSensor,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     accelerometer,  sensorI2CHiTechnicAccel)
#pragma config(Sensor, S4,     compass,        sensorI2CHiTechnicCompass)
#pragma config(Motor,  motorA,          grabber30L,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          grabber30R,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          grabber60L,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     FL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     FR,            tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     BL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     armLift,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    grabber60R,           tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    dump30,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    dump60,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

/* Drivers */
#include "JoystickDriver.c" // include file to "handle" the Bluetooth messages

/* Integers */
int positionGrabber30; // position grabber30 servo
int positionGrabber60; // position grabber60 servo
int positionDump30; // position dump30 servo
int positionDump60; // position dump60 servo

const int GRABBER60RUP = 120; // up position for the grabber60R servo
const int GRABBER60RDOWN = 255; // down position for the grabber60L servo

float x1, y1, x2, y2, LF, RF, LB, RB;
float motorMultiplier;
const int LEGOMOTORTARGET = 160; //sets how far the lego motors move for grabbers

int DEBUGliftPosition = nMotorEncoder[armLift];

void initializeRobot(){
	positionGrabber30 = 0; // position grabber30 servo to up
	positionGrabber60 = 0; // position grabber60 servo to up
	positionDump30 = 0; // position dump30 servo to closed
	positionDump60 = 0; // position dump60 servo to closed
	servo[dump60] = 70; // sets servo to the "closed" position
	servo[dump30] = 190; // sets servo to the "closed" position
	// servo[grabber30] = 120;

	// intialize the 3 motors and 1 servo for grabbers
	servo[grabber60R] = GRABBER60RUP;
	nMotorEncoder[grabber30L] = 0;
	nMotorEncoder[grabber30R] = 0;
	nMotorEncoder[grabber60L] = 0;
	motor[grabber30L] = 0;
	motor[grabber30R] = 0;
	motor[grabber60L] = 0;


	return;
}

/* Start Tube 30 Task */
task tube30() {

	while(true){
		if(joy1Btn(1) && positionGrabber30 == 0) { // if you hit it and the grabber is up, take it down
			positionGrabber30 = 1;  // toggle down
			nMotorEncoderTarget[grabber30L] = LEGOMOTORTARGET;
			nMotorEncoderTarget[grabber30R] = LEGOMOTORTARGET;
			motor[grabber30L] = 50;
			motor[grabber30R] = 50;

			//while(nMotorRunState[grabber30R] != runStateIdle ) { //I hope we don't need to watch both.
			//}
			wait1Msec(300);
			motor[grabber30L] = 1;
			motor[grabber30R] = 1;
			wait1Msec(200);
		}

		else if(joy1Btn(1) && positionGrabber30 == 1) {
			positionGrabber30 = 0; // toggle up
			nMotorEncoderTarget[grabber30L] = LEGOMOTORTARGET;
			nMotorEncoderTarget[grabber30R] = LEGOMOTORTARGET;
			motor[grabber30L] = -50;
			motor[grabber30R] = -50;

			//while(nMotorRunState[grabber30R] != runStateIdle ) { //I hope we don't need to watch both.
			//}
			wait1Msec(300);
			motor[grabber30L] = 0;
			motor[grabber30R] = 0;
			wait1Msec(200);
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}
/* End Tube 30 Task */

/* Start Tube 60 Task */
task tube60() {

	while(true) {
		if(joy1Btn(3) && positionGrabber60 == 0) { // if you hit it and the grabber is up, take it down
			positionGrabber60 = 1;  // toggle down
			servo[grabber60R] = GRABBER60RDOWN;
			nMotorEncoderTarget[grabber60L] = LEGOMOTORTARGET;
			motor[grabber60L] = 50;
		//	while(nMotorRunState[grabber60L] != runStateIdle ) { //I hope we don't need to watch both.
			//}
			wait1Msec(300);
			motor[grabber60L] = 1;
			wait1Msec(200);
	}

	else if(joy1Btn(3) && positionGrabber60 == 1) {
			positionGrabber60 = 0; // toggle up
			servo[grabber60R] = GRABBER60RUP;
			nMotorEncoder[grabber60L] = 0;
			nMotorEncoderTarget[grabber60L] = LEGOMOTORTARGET;
			motor[grabber60L] = -50;
			wait1Msec(300);
			//while(nMotorRunState[grabber60L] != runStateIdle ) { //I hope we don't need to watch both.
			//}
			motor[grabber60L] = 0;
			wait1Msec(200);
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}
/* End Tube 60 Task */

/* Start Door 30 Task */
task door30() {

	while(true) {
		if(joy1Btn(4) && positionDump30 == 0) {
			positionDump30 = 1;  // toggle open

			servo[dump30] = 150;
			wait1Msec(500);
	}

	else if(joy1Btn(4) && positionDump30 == 1) {
			positionDump30 = 0; // toggle closed

			servo[dump30] = 190;
			wait1Msec(500);
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}
/* End Door 30 Task */

/* Start Door 60 Task */
task door60() {

	while(true) {
		if(joy1Btn(2) && positionDump60 == 0) {
			positionDump60 = 1;  // toggle open
			servo[dump60] = 100;
			wait1Msec(500);
		}

		else if(joy1Btn(2) && positionDump60 == 1) {
			positionDump60 = 0; // toggle closed
			servo[dump60] = 70;
			wait1Msec(500);
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}
/* End Door 60 Task */

/* Start Mecanum Wheel Drive */
task drive() {
	motorMultiplier = 0.78; // 25/32
	int minJoy = 12;

	while(true) {

		// Resets movement values
		LF = 0;
		RF = 0;
		LB = 0;
		RB = 0;

		// Get joystick values
		x1 = joystick.joy1_x1 * motorMultiplier;
		x2 = joystick.joy1_x2 * motorMultiplier;
		y1 = joystick.joy1_y1 * motorMultiplier;
		y2 = joystick.joy1_y2 * motorMultiplier;

		// Checking Joystick Threshold
		if (abs(joystick.joy1_x1) < minJoy){
			x1 = 0;
		}

		if (abs(joystick.joy1_x2) < minJoy){
			x2 = 0;
		}

		if (abs(joystick.joy1_y1) < minJoy){
			y1 = 0;
		}

		if (abs(joystick.joy1_y2) < minJoy){
			y2 = 0;
		}

		// Handle Strafing Movement
		LF += x1;
		RF -= x1;
		LB -= x1;
		RB += x1;

		// Handle Regular Movement
		LF += y1;
		RF += y1;
		LB += y1;
		RB += y1;

		// Handle Turning Movement
		LF += x2;
		RF -= x2;
		LB += x2;
		RB -= x2;

		// Apply Finished values to motors.
		motor[FL] = LF;
		motor[FR] = RF;
		motor[BL] = LB;
		motor[BR] = RB;

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}
/* End Mecanum Wheel Drive */

/* Start Arm Slider Task */
task armSlider() {
	motor[armLift] = 0; // the motor is set to not move
	nMotorEncoder[armLift] = 0; //set initial motor spot.
	int positionLift = 0; // the lift starts down
	//1 rotation of wheel = 280 * 4 = 1120 encoder ticks
	const int ARMLIFTTARGET = 2146; //distance the motor rotztes to lift to the 60cm goal.
	DEBUGliftPosition = nMotorEncoder[armLift];

	while(true) {
		if(joy1Btn(5) && positionLift == 0) {
	//		nMotorEncoder[armLift] = 0;
			positionLift = 1;
			motor[armLift] = 20;
			ClearTimer(T1);
			while(nMotorEncoder[armLift] < ARMLIFTTARGET && time1[T1]<3000) { //until you hit the target or 3 secs
				DEBUGliftPosition = nMotorEncoder[armLift];}
			//nMotorEncoderTarget[armLift] = ARMLIFTTARGET;// 1 and 11/12 revs = 1120 + 1026 for Andymark Neverrest motors
			//while(true){
				//DEBUGliftPosition = nMotorEncoder[armLift];}
			motor[armLift]=0;
			wait1Msec(500);
		}

		else if(joy1Btn(5) && positionLift == 1) {
			//nMotorEncoder[armLift] = 0;
			positionLift = 0;
			motor[armLift] = -20; // move slider down
			ClearTimer(T1);
			while(nMotorEncoder[armLift] > 150 && time1[T1]<2500) {
				DEBUGliftPosition = nMotorEncoder[armLift];}
			//
			motor[armLift]=0;
			wait1Msec(500);
		}

	/*	else {
			motor[armLift] = 0; // if the button isn't pressed the slider won't move
		} */

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}

/* Start Main Task */
task main() {
	initializeRobot();

	waitForStart();   // wait for start of tele-op phase

	StartTask(tube30);
	StartTask(tube60);
	StartTask(door30);
	StartTask(door60);
	StartTask(drive);
	StartTask(armSlider);

	while(true) {
		wait10Msec(1000);
	}
}
/* End Main Task */
