#pragma config(Hubs,  S2, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     compass,        sensorI2CCustom)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     accelerometer,  sensorI2CCustom)
#pragma config(Sensor, S4,     irSensor,       sensorHiTechnicIRSeeker1200)
#pragma config(Motor,  motorA,          grabber30L,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          grabber30R,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          grabber60L,    tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S2_C1_1,     FL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C1_2,     FR,            tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     BL,            tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C2_2,     BR,            tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S2_C3_1,     armLift,       tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C3_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S2_C4_1,    grabber60R,           tServoStandard)
#pragma config(Servo,  srvo_S2_C4_2,    servo2,               tServoStandard)
#pragma config(Servo,  srvo_S2_C4_3,    dump30,               tServoStandard)
#pragma config(Servo,  srvo_S2_C4_4,    dump60,               tServoStandard)
#pragma config(Servo,  srvo_S2_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C4_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/* Start Drivers */
#include "JoystickDriver.c" // include file to "handle" the Bluetooth messages
#include "drivers/hitechnic-accelerometer.h" // HiTechnic Acceleration Sensor Driver
#include "drivers/hitechnic-compass.h" // HiTechnic Compass Sensor Driver
#include "drivers/hitechnic-irseeker-v2.h" // HiTechnic IR Seeker V2 Driver

/* End Drivers */

	int xAxis = 0; //acceleration sensor vars
	int yAxis = 0;
	int zAxis = 0;

//DEBUG VARIABLES***********************
int DEBUGencoderValue = 0;
int DEBUGcompass = -1;
int DEBUGaccelx = -1;
int DEBUGaccely = -1;
int DEBUGaccelz = -1;

int initialCompassHeading; // grabs initial heading to be used for compass movement
int desiredCompassHeading;
//int distanceToGo = 12; // distance in inches
//int degreesToTurn = rotations * 360; // how many degrees a wheel rotates

const int ENCODERTICKSPERREVOLUTION = 1120; // 280 * 4
const int GRABBER60RUP = 120; // up position for the grabber60R servo
const int GRABBER60RDOWN = 255; // down position for the grabber60L servo
const int LEGOMOTORTARGET = 160; // sets how far the lego motors move for grabbers

const float FUZZYMATH = 0.59;//multiplier that adjusts encoder math to match reality. TO BE FIXED LATER

const float WHEELDIAMETER = 4.0; // diameter of the wheel in inches
const float WHEELCIRCUMFERENCE = WHEELDIAMETER * PI; // circumference of the wheel
//float  = distanceToGo / circumference; // how many times does the wheel need to rotate to go the total distance

void initializeRobot() {
	DEBUGcompass = 7;
	initialCompassHeading = HTMCsetTarget(compass);//HTMCsetTarget(compass); //grabs init heading to be used for compass movement
	DEBUGcompass = 3;
	desiredCompassHeading = initialCompassHeading;
	HTACreadAllAxes(accelerometer, DEBUGaccelx, DEBUGaccely, DEBUGaccelz);
	// Reset Motor Encoders
  nMotorEncoder[FL] = 0;
  nMotorEncoder[grabber30L] = 0;
	nMotorEncoder[grabber30R] = 0;
	nMotorEncoder[grabber60L] = 0;
	servo[dump60] = 70; // sets servo to the "closed" position
	servo[dump30] = 190; // sets servo to the "closed" position
  servo[grabber60R] = GRABBER60RUP;
	motor[grabber30L] = 0;
	motor[grabber30R] = 0;
	motor[grabber60L] = 0;

  return;
}

void driveOffRamp() {

}

void driveForwardAccel(int minDistanceInches) {
	xAxis = 0;
	yAxis = 0;
	zAxis = 0;

	motor[FL] = 50;
  motor[FR] = 50;
  motor[BL] = 50;
  motor[BR] = 50;

  nMotorEncoder[FL] = 0;
	int encoderTarget = minDistanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE * FUZZYMATH;

	ClearTimer(T2); //use T2 to track how long you've driven
	while(true) {
		HTACreadAllAxes(accelerometer, xAxis, yAxis, zAxis);
		if(xAxis < DEBUGaccelx){//checking lowest X value
			DEBUGaccelx = xAxis;
		}
			if( xAxis > DEBUGaccely){// MAX X value
				DEBUGaccely =xAxis;
			}
    if(xAxis < -200) {//check if you hit something

    	//check that we went the minimum distance (prevents early stops that happen right after start of move
    	if(abs(nMotorEncoder[FL]) > encoderTarget) {
    		//  	hitSomething = true;
    		motor[FL] = 0;
  			motor[FR] = 0;
  			motor[BL] = 0;
  			motor[BR] = 0;
  			return;
  		}
    }
    if(time1(T2) > 4000) { //stop the function if it's been running for more than 10 secs. something has gone wrong
    	motor[FL] = 0;
  		motor[FR] = 0;
  		motor[BL] = 0;
  		motor[BR] = 0;
  		return;
  	}
  }
}

void driveBackwardAccel(int minDistanceInches) {//fix
	xAxis = 0;
	yAxis = 0;
	zAxis = 0;

	// *************************THIS IS JUST A TEST, it will never leave the loop unless you hit something
	motor[FL] = -50;
  motor[FR] = -50;
  motor[BL] = -50;
  motor[BR] = -50;

  nMotorEncoder[FL] = 0;
	int encoderTarget = minDistanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE * FUZZYMATH;

	ClearTimer(T2); //use T2 to track how long you've driven
	while(true) {
		HTACreadAllAxes(accelerometer, xAxis, yAxis, zAxis);
		if(xAxis < DEBUGaccelx){//checking lowest X value
			DEBUGaccelx = xAxis;
		}
			if( xAxis > DEBUGaccely){// MAX X value
				DEBUGaccely =xAxis;
			}
    if(xAxis > 200) {//check if you hit something

    	//check that we went the minimum distance (prevents early stops that happen right after start of move
    	if(abs(nMotorEncoder[FL]) > encoderTarget) {
    		//  	hitSomething = true;
    		motor[FL] = 0;
  			motor[FR] = 0;
  			motor[BL] = 0;
  			motor[BR] = 0;
  			return;
  		}
    }
    if(time1(T2) > 2000) { //stop the function if it's been running for more than 4 secs. something has gone wrong
    	motor[FL] = 0;
  		motor[FR] = 0;
  		motor[BL] = 0;
  		motor[BR] = 0;
  		return;
  	}
  }
}

void driveForward(int distanceInches) {
	nMotorEncoder[FL] = 0;
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE * .45;

	while(abs(nMotorEncoder[FL]) < encoderTarget) {
		DEBUGencoderValue = nMotorEncoder[FL];
    motor[FL] = 50;
    motor[FR] = 50;
    motor[BL] = 50;
    motor[BR] = 50;
  }

	motor[FL] = 0;
	motor[FR] = 0;
  motor[BL] = 0;
  motor[BR] = 0;
}

void driveBackward(int distanceInches) {
  nMotorEncoder[FL] = 0;
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE * .45;

	while(abs(nMotorEncoder[FL]) < encoderTarget) {
    motor[FL] = -40;
    motor[FR] = -40;
    motor[BL] = -40;
    motor[BR] = -40;
  }

	motor[FL] = 0;
	motor[FR] = 0;
  motor[BL] = 0;
  motor[BR] = 0;
}

void grab30() {
  nMotorEncoderTarget[grabber30L] = LEGOMOTORTARGET;
	nMotorEncoderTarget[grabber30R] = LEGOMOTORTARGET;
	motor[grabber30L] = 50;
	motor[grabber30R] = 50;
	wait1Msec(300);
	motor[grabber30L] = 1;
	motor[grabber30R] = 1;
}

void door30() {
  servo[dump30] = 150;
}

void grab60() {
	nMotorEncoderTarget[grabber60L] = LEGOMOTORTARGET;
	motor[grabber60L] = 50;
	servo[grabber60R] = GRABBER60RDOWN;
	wait1Msec(300);
	motor[grabber60L] = 1;
}

void door60() {
	int ARMLIFTTARGET = 2700;

			motor[armLift] = -30;
			ClearTimer(T1);
			while(abs(nMotorEncoder[armLift]) < ARMLIFTTARGET && time1[T1]<3000) {} //until you hit the target or 3 secs
		//		DEBUGliftPosition = nMotorEncoder[armLift];}
			//nMotorEncoderTarget[armLift] = ARMLIFTTARGET;// 1 and 11/12 revs = 1120 + 1026 for Andymark Neverrest motors
			//while(true){
				//DEBUGliftPosition = nMotorEncoder[armLift];}
			motor[armLift]=0;
			wait1Msec(500);


  servo[dump60] = 150;
  wait1Msec(1500);
  servo[dump60] = 70;
  wait1Msec(500);
  motor[armLift] = 20;
  wait1Msec(1000);
  motor[armLift]=0;
}

void rotateCounterClockwise(int rotationDegrees) {

/* RELATIVE HEADING refreshes unreliably. Try a different function.
	while(rotationDegrees > abs(HTMCreadRelativeHeading(compass))) { // rotate counter clockwise until at desired angle
		motor[FL] = -25;
	  motor[FR] = 25;
	  motor[BL] = -25;
	  motor[BR] = 25;
	}

*/
	desiredCompassHeading = (desiredCompassHeading + rotationDegrees + 360) % 360;
	int currentCompassHeading = HTMCreadHeading(compass);
	while(desiredCompassHeading != currentCompassHeading) { // rotate clockwise until at desired angle
	//	DEBUGcompass = HTMCreadHeading(compass);
		DEBUGencoderValue = currentCompassHeading;
		DEBUGcompass = desiredCompassHeading;
		motor[FL] = 30;
		motor[FR] = -30;
		motor[BL] = 30;
		motor[BR] = -30;
		if(abs(currentCompassHeading - desiredCompassHeading) < 2) {
		     break;
		}
		currentCompassHeading = HTMCreadHeading(compass);
	}
				DEBUGencoderValue = currentCompassHeading;
		DEBUGcompass = desiredCompassHeading;


	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void rotateCounterClockwiseNoCompass(int timeMSec) {
	ClearTimer(T3);
	while(time1(T3) < timeMSec) {
		motor[FL] = -40;
		motor[FR] = 40;
		motor[BL] = -40;
		motor[BR] = 40;
		wait1Msec(10);
	}
	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void rotateClockwiseNoCompass(int timeMSec) {
	ClearTimer(T3);
	while(time1(T3) < timeMSec) {
		motor[FL] = 40;
		motor[FR] = -40;
		motor[BL] = 40;
		motor[BR] = -40;
		wait1Msec(10);
	}
	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void rotateClockwise(int rotationDegrees) {//IF rotateCCW works, switch this code to similar to THAT CODE
	desiredCompassHeading = (desiredCompassHeading + rotationDegrees + 360) % 360;
	int currentCompassHeading = HTMCreadHeading(compass);
	while(desiredCompassHeading != currentCompassHeading) { // rotate clockwise until at desired angle
	//	DEBUGcompass = HTMCreadHeading(compass);
		if(currentCompassHeading + 1 == desiredCompassHeading ||
				currentCompassHeading + 2 == desiredCompassHeading ||
				currentCompassHeading - 1 == desiredCompassHeading ||
				currentCompassHeading - 2 == desiredCompassHeading) {
					break;//if you are within 2 degrees of desired heading, STOP. This is because the compass does not refresh fast enough and skips the desired heading sometimes
		}
		DEBUGencoderValue = currentCompassHeading;
		DEBUGcompass = desiredCompassHeading;
		motor[FL] = 40;
		motor[FR] = -40;
		motor[BL] = 40;
		motor[BR] = -40;
		currentCompassHeading = HTMCreadHeading(compass);
	}

	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void strafeLeft(int distanceInches) {
	nMotorEncoder[BL] = 0;
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE *1.25;
	while(abs(nMotorEncoder[BL]) < encoderTarget) {
		DEBUGencoderValue = nMotorEncoder[BL];
    motor[FL] = -60;
    motor[FR] = 60;
    motor[BL] = 60;
    motor[BR] = -60;
  }

	motor[FL] = 0;
	motor[FR] = 0;
  motor[BL] = 0;
  motor[BR] = 0;
}

void strafeRight(int distanceInches) {

	nMotorEncoder[BL] = 0;
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE * 1.25;

	while(abs(nMotorEncoder[BL]) < encoderTarget) {
		DEBUGencoderValue = nMotorEncoder[BL];
    motor[FL] = 100;
    motor[FR] = -100;
    motor[BL] = -100;
    motor[BR] = 100;
  }

	motor[FL] = 0;
	motor[FR] = 0;
  motor[BL] = 0;
  motor[BR] = 0;
}

task main()
{
  initializeRobot();

  waitForStart(); // wait for the beginning of autonomous phases

//FUNCTIONS TO TEST, CHECK THEM OFF WHEN THEY LOOK ACCURATE********************************************************
	//driveForward(24);//DONE WORKS GOOD ENOUGH
//	driveBackward(24);//PERFECT!
  //rotateCounterClockwiseNoCompass(900); WORKS
//  rotateClockwise(45);
  //strafeLeft(24); WORKS
 // strafeRight(24); WORKS
  //driveForwardAccel(24);
 // driveBackwardAccel(24); //WORKS BUT NEEDS TO BE FAR ENOUGH AWAY

 // grab30();

 // door30();
  //grab60();
 // door60();





  driveForward(24);
  driveForward(24);
  driveForward(24);
  driveForward(8);//all 3 to drive off rampdriveForward(24);
  wait1Msec(500);
  strafeLeft(26);//move left to position to 30cm goal
  wait1Msec(500);
  driveForwardAccel(24); // drive to run into 30cm
  wait1Msec(500);
  grab30();//grab the 30cm
  wait1Msec(500);
  door30();//dump into 30cm
  wait1Msec(500);
  driveBackward(14);//backup 2 feet
  wait1Msec(500);
  rotateCounterClockwiseNoCompass(950);//rotate to align for 60cm goal
  wait1Msec(500);
  driveBackwardAccel(10); //run into 60cm goal
  wait1Msec(500);
  grab60();//grab the 60cm
  wait1Msec(500);
  door60();//dump into the 60
  wait1Msec(500);
  driveForward(24);//drive back I"M GUESSING
  wait1Msec(500);
  rotateCounterClockwiseNoCompass(900); //rotate so 30cm is towards parking zone
  wait1Msec(500);
  driveForward(24);
  driveForward(24);
  driveForward(24);
  driveForward(24);
  driveForward(24);//drive alongside the ramp towards parking zone, with 30cm near wall
  wait1Msec(500);
  strafeRight(12); //go right to make room to rotate
  wait1Msec(500);
  rotateClockwiseNoCompass(800); //angle to drive into park zone
  wait1Msec(500);
  driveForward(24); //should end with both goals in parking zone



  wait1Msec(30000); //wait for end of Auto
}
