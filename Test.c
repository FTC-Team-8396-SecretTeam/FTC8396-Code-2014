#pragma config(Hubs,  S1, HTServo,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C2_1,     armLift,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C1_1,    testServo,            tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

int DEBUGliftPosition = nMotorEncoder[armLift];
int armLiftTarget = 2000;

void initializeRobot() {

}

task main()
{
	waitForStart();

	while (true) {

		/* Start Servo */
		if (joy1Btn(2) == 0) { // servo up
			servo[testServo] = 100;
		}

		else if (joy1Btn(2) == 1) { // servo down
			servo[testServo] = 70;
		}

		/* Start Motor */
		if(joy1Btn(5)) { // motor down
			motor[armLift] = -70;
		}

		else if(joy1Btn(7) && abs(nMotorEncoder[armLift]) < armLiftTarget) { // motor up
			motor[armLift] = 70;
		}

		else { // motor doesn't move
			motor[armLift] = 0;
		}

		wait1Msec(10);
	}
}