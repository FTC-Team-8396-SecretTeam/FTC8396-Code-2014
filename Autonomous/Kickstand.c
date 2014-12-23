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

#include "JoystickDriver.c"  //Include file to "handle" the Bluetooth messages.

void initializeRobot()
{
  return;
}

task main()
{
  initializeRobot();

  waitForStart(); // Wait for the beginning of autonomous phase.

  	motor[FR] = 50;
	  motor[FL] = -50;
	  motor[BR] = 50;
	  motor[BL] = -50;
	  wait1Msec(3000);
		motor[FR] = 0;
	  motor[FL] = 0;
	  motor[BR] = 0;
	  motor[BL] = 0;

  while(1 == 1)
  {
    if(SensorValue[IRSeeker2] == 4)
    {
      motor[FR] = 0;
		  motor[FL] = 0;
		  motor[BR] = 0;
		  motor[BL] = 0;
    }

    if(SensorValue[IRSeeker2] > 5)
    {
      motor[FR] = 50;
		  motor[FL] = -50;
		  motor[BR] = 50;
		  motor[BL] = -50;
    }

    if(SensorValue[IRSeeker2] < 5)
    {
      motor[FR] = 50;
		  motor[FL] = -50;
		  motor[BR] = 50;
		  motor[BL] = -50;
    }
  }
}
