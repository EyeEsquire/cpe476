#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *frontLeftM = AFMS.getMotor(1);
Adafruit_DCMotor *frontRightM = AFMS.getMotor(2);
Adafruit_DCMotor *backLeftM = AFMS.getMotor(3);
Adafruit_DCMotor *backRightM = AFMS.getMotor(4);

int counter = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
}

void loop() {
  uint8_t i;
  uint8_t k = 0, l = 0;
  Serial.print("Executing: STRAIGHT LINE!!!\n\n");

  //frontRightM->setSpeed(255);
  //frontLeftM->setSpeed(100);
  //backLeftM->setSpeed(100);
  //backRightM->setSpeed(255);

  // Run Forward -----------------------------------
  backLeftM->run(FORWARD);
  frontRightM->run(FORWARD);
  frontLeftM->run(FORWARD);
  backRightM->run(FORWARD);

  for(i=0; i<255; i++) {
    frontRightM->setSpeed(i);
    frontLeftM->setSpeed(i);
    backLeftM->setSpeed(i);
    backRightM->setSpeed(i);
    delay(10);
  }

  delay(2000);

  for(i=255; i!=0; i--) {
    frontRightM->setSpeed(i);
    frontLeftM->setSpeed(i);
    backLeftM->setSpeed(i);
    backRightM->setSpeed(i);
    delay(10);
  }
 
  delay(500);
  frontRightM->run(RELEASE);
  frontLeftM->run(RELEASE);
  backRightM->run(RELEASE);
  backLeftM->run(RELEASE);
}
