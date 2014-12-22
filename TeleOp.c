#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     irSensor,       sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     accelerometer,  sensorI2CHiTechnicAccel)
#pragma config(Sensor, S4,     compass,        sensorI2CHiTechnicCompass)
#pragma config(Motor,  mtr_S1_C1_1,     FL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     FR,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     BL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     armLift,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    grabber30,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_2,    grabber60,            tServoStandard)
#pragma config(Servo,  srvo_S1_C4_3,    dump30,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_4,    dump60,               tServoStandard)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)

/* Drivers */
#include "JoystickDriver.c"
#include "drivers/hightechnic-irseeker-v2.h"
#include "drivers/hightechnic-accelerometer.h"
#include "drivers/hightechnic-compass.h"

int positionGrabber30; // position grabber30 servo
int positionGrabber60; // position grabber60 servo
int positionDump30; // position dump30 servo
int positionDump60; // position dump60 servo

void initializeRobot(){
	positionGrabber30 = 0; // position grabber30 servo to up
	positionGrabber60 = 0; // position grabber60 servo to up
	positionDump30 = 0; // position dump30 servo to closed
	positionDump60 = 0; // position dump60 servo to closed

	return;
}

/* Start Tube Task */
task tube30 (){

	while(true){
		if(joy1Btn(1) && positionGrabber30 == 0){ // if you hit it and the grabber is up, take it down
			positionGrabber30 = 1;  // toggle down
			servo[grabber30]= 255;
			servo[grabber30]=0;
			wait1Msec(500);
	}

	else if(joy1Btn(1) && positionGrabber30 == 1){
			positionGrabber30 = 0; // toggle up
			servo[grabber30]=0;
			servo[grabber30]= 255;
			wait1Msec(500);
		}

		wait1Msec(10);
	}
}
/* End Tube 30 Task */

/* Start Tube 60 Task */
task tube60 (){

	while(true){
		if(joy1Btn(3) && positionGrabber60 == 0){ // if you hit it and the grabber is up, take it down
			positionGrabber60 = 1;  // toggle down
			servo[grabber60]= 255;
			servo[grabber60]=0;
			wait1Msec(500);
	}

	else if(joy1Btn(3) && positionGrabber60 == 1){
			positionGrabber60 = 0; // toggle up
			servo[grabber60]=0;
			servo[grabber60]= 255;
			wait1Msec(500);
		}

		wait1Msec(10);
	}
}
/* End Tube 60 Task */

/* Start Door 30 Task */
task door30 (){

	while(true){
		if(joy1Btn(4) && positionDump30 == 0){
			positionDump30 = 1;  // toggle open
			servo[dump30]= 255;
			servo[dump30]=0;
			wait1Msec(500);
	}

	else if(joy1Btn(4) && positionDump30 == 1){
			positionDump30 = 0; // toggle closed
			servo[dump30]=0;
			servo[dump30]= 255;
			wait1Msec(500);
		}

		wait1Msec(10);
	}
}
/* End Door 30 Task */

/* Start Door 60 Task */
task door60 (){

	while(true){
		if(joy1Btn(2) && positionDump60 == 0){
			positionDump60 = 1;  // toggle open
			servo[dump60]= 255;
			servo[dump60]=0;
			wait1Msec(500);
	}

	else if(joy1Btn(2) && positionDump60 == 1){
			positionDump60 = 0; // toggle closed
			servo[dump60]=0;
			servo[dump60]= 255;
			wait1Msec(500);
		}

		wait1Msec(10);
	}
}
/* End Door 60 Task */

/* Start basic mecanum wheel drive */
task drive(){
	float x1, y1, x2, y2, LF, RF, LB, RB;
	float motorMultiplier = 25/32;
	int minJoy = 12;

	while(true){

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
		if (joystick.joy1_x1 < minJoy){
			x1 = 0;
		}

		if (joystick.joy1_x2 < minJoy){
			x2 = 0;
		}

		if (joystick.joy1_y1 < minJoy){
			y1 = 0;
		}

		if (joystick.joy1_y2 < minJoy){
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

task sliderLift(){
	motor[armLift]=0;

	while(true){
		if(joy1Btn(5)){
			motor[armLift]=20;
		}

		else if(joy1Btn(7)){
			motor[armLift]=-20;
		}

		else{
			motor[armLift]=0;
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}

/* task rotateContainer(){
	motor[BallContainer]=0;
	while(true){

		if(joy1Btn(4)){
			motor[BallContainer]=15;
		}

		else if(joy1Btn(6)){
			motor[BallContainer]=-15;
		}

		else{
			motor[BallContainer]=0;
		}

		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
} */

task main(){
	initializeRobot();

	waitForStart();   // wait for start of tele-op phase

	StartTask (tube30);
	StartTask (tube60);
	StartTask (door30);
	StartTask (door60);
	StartTask (drive);
	StartTask (sliderLift);

	while (true)
	{
		wait10Msec(1000);
	}
}
