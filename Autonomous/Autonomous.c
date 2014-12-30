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

/* Start Drivers */
#include "JoystickDriver.c" // include file to "handle" the Bluetooth messages
#include "drivers/hitechnic-accelerometer.h" // HiTechnic Acceleration Sensor Driver
#include "drivers/hitechnic-compass.h" // HiTechnic Compass Sensor Driver
#include "drivers/hitechnic-irseeker-v2.h" // HiTechnic IR Seeker V2 Driver
/* End Drivers */

//bool ReadSensorHTAccel(S3, int &x, int &y, int &z)

int initialCompassHeading; // grabs initial heading to be used for compass movement
int desiredCompassHeading;
//int distanceToGo = 12; // distance in inches
//int degreesToTurn = rotations * 360; // how many degrees a wheel rotates

const int ENCODERTICKSPERREVOLUTION = 1120; // 280 * 4
const int GRABBER60RUP = 120; // up position for the grabber60R servo
const int GRABBER60RDOWN = 255; // down position for the grabber60L servo
const int LEGOMOTORTARGET = 160; // sets how far the lego motors move for grabbers

const float WHEELDIAMETER = 4.0; // diameter of the wheel in inches
const float WHEELCIRCUMFERENCE = WHEELDIAMETER * PI; // circumference of the wheel

void initializeRobot() {
	initialCompassHeading = HTMCsetTarget(compass); //grabs initial heading to be used for compass movement
	desiredCompassHeading = initialCompassHeading;

	// reset motor encoders
  nMotorEncoder[FL] = 0;
  nMotorEncoder[grabber30L] = 0;
	nMotorEncoder[grabber30R] = 0;
	nMotorEncoder[grabber60L] = 0;

	// set grabbers to up position
	motor[grabber60L] = 0;
  servo[grabber60R] = GRABBER60RUP;
	motor[grabber30L] = 0;
	motor[grabber30R] = 0;

  return;
}

void driveOffRamp() {

}

void driveForwardAccel(int minDistanceInches) {

	int xAxis = 0;
	int yAxis = 0;
	int zAxis = 0;

	// THIS IS JUST A TEST, won't leave loop unless hit
	motor[FL] = 20;
  motor[FR] = 20;
  motor[BL] = 20;
  motor[BR] = 20;
	HTACreadAllAxes(S3, xAxis, yAxis, zAxis);

	while(true) {
    if(xAxis < -100) { // check if you hit something
  //  hitSomething = true;
    	motor[FL] = 0;
  		motor[FR] = 0;
  		motor[BL] = 0;
  		motor[BR] = 0;
  		return;
  	}
  }
}

void driveBackwardAccel(int t) { // fix

}

void driveForward(int distanceInches) {
	nMotorEncoder[FL] = 0;
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE;

	while(nMotorEncoder[FL] < encoderTarget) {
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
	int encoderTarget = distanceInches * ENCODERTICKSPERREVOLUTION / WHEELCIRCUMFERENCE;

	while(nMotorEncoder[FL] < encoderTarget) {
    motor[FL] = -50;
    motor[FR] = -50;
    motor[BL] = -50;
    motor[BR] = -50;
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

void tube30() {
  servo[dump30] = 150;
}

void grab60() {
	nMotorEncoderTarget[grabber60L] = LEGOMOTORTARGET;
	motor[grabber60L] = 50;
	servo[grabber60R] = GRABBER60RDOWN;
	wait1Msec(300);
	motor[grabber60L] = 1;
}

void tube60() {
  servo[dump60] = 150;
}

void rotateCounterClockwise(int rotationDegrees) {
	desiredCompassHeading = (desiredCompassHeading - rotationDegrees + 360) % 360;

	while(desiredCompassHeading != HTMCreadHeading(compass)) { // rotate counter clockwise until at desired angle
		motor[FL] = -20;
	  motor[FR] = 20;
	  motor[BL] = -20;
	  motor[BR] = 20;
	}

	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void rotateClockwise(int rotationDegrees) {
	desiredCompassHeading = (initialCompassHeading - rotationDegrees + 360) % 360;

	while(desiredCompassHeading != HTMCreadHeading(compass)) { // rotate clockwise until at desired angle
		motor[FL] = 20;
		motor[FR] = -20;
		motor[BL] = 20;
		motor[BR] = -20;
	}

	motor[FL] = 0;
 	motor[FR] = 0;
	motor[BL] = 0;
	motor[BR] = 0;
}

void strafeRight(int t) { // fix
  motor[FL] = 50;
  motor[FR] = -50;
  motor[BL] = -50;
  motor[BR] = 50;
  wait1Msec(t);
  motor[FL] = 0;
  motor[FR] = 0;
  motor[BL] = 0;
  motor[BR] = 0;
}

task main() {
  initializeRobot();

  waitForStart(); // wait for the beginning of autonomous phase

  driveOffRamp(); // drive forward using z axis from accelerometer
  rotateCounterClockwise(45); // rotate counter clockwise 45 degrees using compass sensor
  driveForward(33.9411); // drive forward 33.95 inches using motor encoders
  rotateClockwise(45); // rotate clockwise 45 degrees using compass sensor
  driveForwardAccel(2); // use accelerometer to know robot position/align robot
  grab30(); // grab the 30 centimeter rolling goal
  tube30(); // score into the 30 centimeter goal
  driveBackward(24); // drive backward 24 inches using motor encoders
  rotateCounterClockwise(90); // rotate counter clockwise 90 degrees using compass sensor
  driveBackwardAccel(2); // use accelerometer to know robot position/align robot
  grab60(); // grab the 60 centimeter rolling goal
  tube60(); // score into the 60 centimeter goal
  driveForward(12); // drive forward 12 inches using motor encoders
  rotateCounterClockwise(90); // rotate counter clockwise 90 degrees using compass sensor
  driveForward(12); // drive forward ? inches using motor encoders
  strafeRight(6); // strafe right ? inches using motor encoders
  rotateClockwise(45); // rotate clockwise 45 degrees using compass
  driveForward(24); // drive forward 24 inches using motor encoders
}
