// Authors: Keanan Gabertan, Vincent Vuong
// CPE476 - Midterm
// This code will have the rover perform a 
// set of maneuvers and calculate its dead reckoning
// based on its odometry

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <DeadReckoner.h>

// ENCODER PINS
// TWO for each motor
// A - records every time there is a rising signal
// B - records forward or reverse
// ENCODER A is orange on motor encoder
#define FR_ENCODER_A 14
//#define FR_ENCODER_B 15
#define FL_ENCODER_A 11
//#define FL_ENCODER_B 10
#define BR_ENCODER_A 22
//#define BR_ENCODER_B 21
#define BL_ENCODER_A 2
//#define BL_ENCODER_B 3


// MEASUREMENTS
// The units for all measurements must be consistent. 
// You can use any length unit as desired.
#define RADIUS 40 // wheel radius in mm
#define LENGTH 150 // wheel base length in mm

// Ticks per rev based on its 1:45 gear ratio and 3 magnetometers
#define TICKS_PER_REV 135

// TIME INTERVALS
#define POSITION_COMPUTE_INTERVAL 50 // milliseconds
#define SEND_INTERVAL 100            // milliseconds


// Number of left and right tick counts on the encoder.
volatile unsigned long leftTicks, rightTicks;

// Previous times for computing elapsed time.
unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

// Instantiate motorshield object and motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *front_left = AFMS.getMotor(1);
Adafruit_DCMotor *front_right = AFMS.getMotor(2);
Adafruit_DCMotor *back_left = AFMS.getMotor(3);
Adafruit_DCMotor *back_right = AFMS.getMotor(4);

// Instantiate deadReckoner object
DeadReckoner deadReckoner(&leftTicks, &rightTicks, TICKS_PER_REV, RADIUS, LENGTH);

// Manuever functions
void goForward()
{
    int i=0;
    back_left->run(FORWARD);
    front_right->run(FORWARD);
    front_left->run(FORWARD);
    back_right->run(FORWARD);

    for (i = 0; i < 255; i++)
    {
        front_right->setSpeed(i);
        front_left->setSpeed(i);
        back_left->setSpeed(i);
        back_right->setSpeed(i);
        delay(10);
    }
}

void turnLeft()
{
  int i=0;
  front_right->run(FORWARD);
  back_right->run(FORWARD);
  //front_left->run(BACKWARD);
  //back_left->run(BACKWARD);
  front_left->setSpeed(15);
  back_left->setSpeed(10);

  for (i=0; i<255; i++) {
      front_right->setSpeed(i);
      back_right->setSpeed(i);

      delay(10);
  }  
}

void turnRight()
{
  int i=0;
  //front_right->run(BACKWARD);
  //back_right->run(BACKWARD);
  front_left->run(FORWARD);
  back_left->run(FORWARD);
  front_right->setSpeed(15);
  back_right->setSpeed(10);

  for (i=0; i<255; i++) {
      front_left->setSpeed(i);
      back_left->setSpeed(i);
      delay(10);
  }  
}

void calculateDR() {
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
		// Computes the new angular velocities and uses that to compute the new position.
		// The accuracy of the position estimate will increase with smaller time interval until a certain point.
		deadReckoner.computePosition();
		prevPositionComputeTime = millis();
	}

	if (millis() - prevSendTime > SEND_INTERVAL) {
		// Cartesian coordinate of latest location estimate.
		// Length unit correspond to the one specified under MEASUREMENTS.
		double x = deadReckoner.getX();
		double y = deadReckoner.getY();

		// Left and right angular velocities.
		double wl = deadReckoner.getWl();
		double wr = deadReckoner.getWr();

		// getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
		// This angle is set initially at zero before the robot starts moving.
		double theta = deadReckoner.getTheta();

		// Total distance robot has troubled.
		double distance = sqrt(x * x + y * y);

		Serial.print("x: "); Serial.print(x);
		Serial.print("\ty: "); Serial.print(y);
		Serial.print("\twl: "); Serial.print(wl);
		Serial.print("\twr: "); Serial.print(wr);
		Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
		Serial.print("\tdist: "); Serial.println(distance);

		prevSendTime = millis();
	}
}

// If motor encoders interrupt, increase tick count
void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

void attachInterrupts() {
	attachInterrupt(digitalPinToInterrupt(FL_ENCODER_A), pulseLeft, RISING);
	attachInterrupt(digitalPinToInterrupt(FR_ENCODER_A), pulseRight, RISING);
}

void setup() {
  //attachInterrupts();
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Robot DeadReckoning Test");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  front_right->setSpeed(10);
  front_left->setSpeed(10);
  back_right->setSpeed(10);
  back_left->setSpeed(10);
  
  // turn on motor
  front_right->run(RELEASE);
  front_left->run(RELEASE);
  back_right->run(RELEASE);
  back_left->run(RELEASE);
  attachInterrupts();
}

void loop() {
  uint8_t i=0;

  Serial.print("Executing: Evasive Manuevers\n\n");
  calculateDR();
  
  if(i==0) {
    i++;
    // Time step: 0 seconds --------------------------
    // Run Forward for 5 sec -------------------------
    goForward();  
    delay(5000);
    calculateDR();
    // Time step: 5 seconds --------------------------
    // Turn Left for 4.5 sec -------------------------
    turnLeft();
    delay(4500);
    calculateDR(); 
    // Time step: 9.5 seconds ------------------------
    // Run Forward for 3 sec -------------------------
    goForward();  
    delay(3000);
    calculateDR();
    // Time step: 12.5 seconds -----------------------
    // Turn Right for 3.5 seconds --------------------
    turnRight();
    delay(3500);
    calculateDR();
    // Time step: 16 seconds -------------------------

    front_right->run(RELEASE);
    front_left->run(RELEASE);
    back_right->run(RELEASE);
    back_left->run(RELEASE);
  }
}
