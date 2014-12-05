#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     BL,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     FR,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"  // Include file to "handle" the Bluetooth messages.

float initial; float heading; float radheading; int lastTime = 0;//initial is the initial Gyro readings. heading is the robot's yaw
float FLset; float FRset; // these stand for front right set and front left set, which refer to wheels on the robot
//FL set is the front-left motor and the back-right motor, and FR set is the other two motors
float joyAngle; // angle of the first joystick

void moveDirection(float angle, float magnitude){ // sets the motor sets to move at certain speeds depending on the vector given
	FLset = magnitude * (cos(angle) + sin(angle));
	FRset = magnitude * (sin(angle) - cos(angle));
}

void initializeRobot(){
	// Finds average base gyro reading
	for(int i = 0; i < 100; i++){ //sums up first hundred gyro readings
		initial += SensorValue[S4];
		wait10Msec(1);
	}
	initial = initial / 100; //divides by 100 to find the average reading
  return;
}

// This is the superdrive task. If you can think of any better names, please tell me :P
// This task incorporates two modes, regular mecanum driving and free-spinning mode
task superDrive(){
	float x1,y1,x2,y2,LF,RF,LB,RB= 0;
	int minJoy = 12;
	float turning;
	float mag; // magnitude of the joystick vector
	float initialHeading = radheading; float calcHeading = radheading; // sets a base heading for the
	float movementAmount, turningAmount, totalAmount; // for apportioning power to turning and moving
	while(true){
		// Get joystick values
		x1 = joystick.joy1_x1 * .5;y1 = joystick.joy1_y1 * .5;
		x2 = joystick.joy1_x2 * .5;y2 = joystick.joy1_y2 * .5;
		// function for making new initial heading
		if (joy1Btn(5)==1){
			initialHeading = radheading;
		}
		// starts free-spinning mode
		if (joy1Btn(6)==1){
			// find joystick vector angle
			joyAngle = atan2(y1, x1);
			// find joystick vector magnitude
			mag = sqrt(x1*x1+y1*y1)/2;
			if (mag>64)
				mag = 64;
			// get calculated heading
			calcHeading = radheading - initialHeading;
			// find the direction needed to move
	    	moveDirection(joyAngle + calcHeading, mag);
	    	// fix movement drifting
	    	if (abs(joystick.joy1_x1)<minJoy&&abs(joystick.joy1_y1)<minJoy/*&&abs(joystick.joy1_x2)<minJoy*/){
				FLset=0;FRset=0;
			}
			// find turning magnitude
			turning = x2/2;
			//fix turning drifting
			if (abs(joystick.joy1_x2)<minJoy){
				turning = 0;
			}
			// apportion motor capacity to movement and turning
			// TODO make this better... although that will be difficult
			totalAmount = 1 + turning + mag*3;
			movementAmount = (mag*3)/totalAmount;
			turningAmount = turning/totalAmount;
			// Apply finished values to motors
	  		motor[FL] = FLset*movementAmount+turning*turningAmount;
			motor[FR] = FRset*movementAmount-turning*turningAmount;
			motor[BL] = FRset*movementAmount+turning*turningAmount;
			motor[BR] = FLset*movementAmount-turning*turningAmount;
		} else {
			// Resets movement values
			LF = 0;RF = 0;LB = 0;RB = 0;
			// Get joystick values
			x1 = joystick.joy1_x1 * .5;y1 = joystick.joy1_y1 * .5;
			x2 = joystick.joy1_x2 * .5;y2 = joystick.joy1_y2 * .5;
			// Handle Strafing Movement
			LF += x1;RF -= x1;LB -= x1;RB += x1;
			// Handle Regular Movement
			LF += y1;RF += y1;LB += y1;RB += y1;
			// Handle Turning Movement
			LF += x2;RF -= x2;LB += x2;RB -= x2;
			if (abs(joystick.joy1_x1)<minJoy&&abs(joystick.joy1_y1)<minJoy&&abs(joystick.joy1_x2)<minJoy){
				LF = 0;RF = 0;LB = 0;RB = 0;
			}
			// Apply Finished values to motors.
			motor[FL] = LF;
			motor[FR] = RF;
			motor[BL] = LB;
			motor[BR] = RB;
		}
		wait10Msec(1);
	}
}

//basic mecanum wheel drive
task drive(){
	float x1,y1,x2,y2,LF,RF,LB,RB= 0;
	int minJoy = 12;
	while(true){
		// Resets movement values
		LF = 0;RF = 0;LB = 0;RB = 0;
		// Get joystick values
		x1 = joystick.joy1_x1 * .5;y1 = joystick.joy1_y1 * .5;
		x2 = joystick.joy1_x2 * .5;y2 = joystick.joy1_y2 * .5;
		// Handle Strafing Movement
		LF += x1;RF -= x1;LB -= x1;RB += x1;
		// Handle Regular Movement
		LF += y1;RF += y1;LB += y1;RB += y1;
		// Handle Turning Movement
		LF += x2;RF -= x2;LB += x2;RB -= x2;
		if (abs(joystick.joy1_x1)<minJoy&&abs(joystick.joy1_y1)<minJoy&&abs(joystick.joy1_x2)<minJoy){
			LF = 0;RF = 0;LB = 0;RB = 0;
		}
		// Apply Finished values to motors.
		motor[FL] = LF;
		motor[FR] = RF;
		motor[BL] = LB;
		motor[BR] = RB;
		wait1Msec(10); // necessary if using task control to allow for other tasks to run
	}
}

// This task finds the heading of the robot using the gyro.
// Keep in mind that this needs the code written in the initializeRobot() function to work

// This is a task mostly for debugging. It displays values for the heading and joystick angle
// This task is not essential and can be deleted
task display(){
	while (true){
		eraseDisplay();
		//nxtDisplayCenteredTextLine(0, "Color: %d", c);
		nxtDisplayCenteredTextLine(0, "Heading: %d", heading);
		nxtDisplayCenteredTextLine(1, "joyAngle: %d", joyAngle);
		wait1Msec(20);
	}
}

task main(){
  initializeRobot();

  waitForStart();   // wait for start of tele-op phase
  StartTask(display);
  StartTask(heading);
  //StartTask(drive);
  //StartTask(superDrive);
  while (true)
  {
	  wait10Msec(1000);
  }
}
