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

/* Start Drivers */
#include "JoystickDriver.c" // include file to "handle" the Bluetooth messages
#include "Drivers/hightechnic-accelerometer.h" // HiTechnic Acceleration Sensor Driver
#include "Drivers/hightechnic-compass.h" // HiTechnic Compass Sensor Driver
#include "Drivers/hightechnic-irseeker-v2.h" // HiTechnic IR Seeker V2 Driver
/* End Drivers */

int distanceToGo = 12; // distance in inches
int degreesToTurn = rotations * 360; // how many degrees a wheel rotates

float diameterOfWheel = 4; // diameter of the wheel in inches
float circumference = diameterOfWheel * PI; // circumference of the wheel
float rotations = distanceToGo / circumference; // how many times does the wheel need to rotate to go the total distance

void initializeRobot() {

  // Reset Motor Encoders
  nMotorEncoder[FL] = 0;
  nMotorEncoder[FR] = 0;
  nMotorEncoder[BL] = 0;
  nMotorEncoder[BR] = 0;

  return;
}

void driveOffRamp() {

}

void driveForwardAccel(int t) {

}

void driveBackwardAccel(int t) {

}

void driveForward(int t) {
  if(nMotorEncoder == t) {
    motor[FL] = 0;
    motor[FR] = 0;
    motor[BL] = 0;
    motor[BR] = 0;
  }

  else {
    motor[FL] = -50;
    motor[FR] = -50;
    motor[BL] = -50;
    motor[BR] = -50;
  }
}

void driveBackward(int t) {
  if(nMotorEncoder == t) {
    motor[FL] = 0;
    motor[FR] = 0;
    motor[BL] = 0;
    motor[BR] = 0;
  }

  else {
    motor[FL] = -50;
    motor[FR] = -50;
    motor[BL] = -50;
    motor[BR] = -50;
  }
}

void grab30() {
  servo[grab30] = 255;
	servo[grab30] = 0;
	wait1Msec(500);
}

void dump30() {
  servo[dump30] = 255;
	servo[dump30] = 0;
	wait1Msec(500);
}

void grab60() {
  servo[grab60] = 255;
	servo[grab60] = 0;
	wait1Msec(500);
}

void dump60() {
  servo[dump60] = 255;
	servo[dump60] = 0;
	wait1Msec(500);
}

void rotateCounterClockwise(int d) {

}

void rotateClockwise(int d) {

}

void strafeRight(int t) {
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

task main()
{
  initializeRobot();

  waitForStart(); // wait for the beginning of autonomous phase

  driveOffRamp(); // drive forward using z axis from accelerometer
  rotateCounterClockwise(45); // rotate counter clockwise 45 degrees using compass sensor
  driveForward(); // drive forward 33.95 inches using motor encoders
  rotateClockwise(45); // rotate clockwise 45 degrees using compass sensor
  driveForwardAccel(); // use accelerometer to know robot position/align robot
  grab30(); // grab the 30 centimeter rolling goal
  dump30(); // score into the 30 centimeter goal
  driveBackward(); // drive backward 24 inches using motor encoders
  rotateCounterClockwise(90); // rotate counter clockwise 90 degrees using compass sensor
  driveBackwardAccel(); // use accelerometer to know robot position/align robot
  grab60(); // grab the 60 centimeter rolling goal
  dump60(); // score into the 60 centimeter goal
  driveForward(); // drive forward 12 inches using motor encoders
  rotateCounterClockwise(90); // rotate counter clockwise 90 degrees using compass sensor
  driveForward(); // drive forward ? inches using motor encoders
  strafeRight(); // strafe right ? inches using motor encoders
  rotateClockwise(45); // rotate clockwise 45 degrees using compass
  driveForward(); // drive forward 24 inches using motor encoders
}
