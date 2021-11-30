#include <ros.h>
#include <std_msgs/Int16.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 11
#define ENC_IN_RIGHT_A 14
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 10
#define ENC_IN_RIGHT_B 15

// #define ENC_IN_BLEFT_A 
// #define ENC_IN_BRIGHT_A

// #define ENC_IN_BLEFT_B
// #define ENC_IN_BRIGHT_B

// Motor encoder output pulses per 135 degree revolution
#define ENC_COUNT_REV 270

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Int16 rpmRight;
ros::Publisher rightPubRPM("RPM_RIGHT", &rpmRight);

std_msgs::Int16 rpmLeft;
ros::Publisher leftPubRPM("RPM_LEFT", &rpmLeft);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
// Variables for angular velocity measurement
float rpm_right = 0;
float rpm_left = 0;

float ang_velocity_right = 0;
float ang_velocity_right_deg = 0;
float ang_velocity_left = 0;
float ang_velocity_left_deg = 0;

const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


// Increment the number of ticks
// front right direction is opposite in our configuration
// direction is changed accordingly now
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = true; // Reverse
  }
  else {
    Direction_right = false; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
// Get RPM and angular velocity
void get_velocity() {
  rpm_right = (float)(right_wheel_tick_count * 60 / ENC_COUNT_REV);
  rpm_left = (float)(left_wheel_tick_count * 60 / ENC_COUNT_REV);

  rpmRight.data = rpm_right;
  rpmLeft.data = rpm_left;

  ang_velocity_right = rpm_right * rpm_to_radians;
  ang_velocity_right_deg = ang_velocity_right * rad_to_deg;

  ang_velocity_left = rpm_left * rpm_to_radians;
  ang_velocity_left_deg = ang_velocity_left * rad_to_deg;
}

void setup() {

 Serial.begin(57600);   // 0 mean dont care about it teensy manage it
 //while (!Serial && (millis() < 5000)) ; // wait up to 5 seconds for host to have serial object ready... **** 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 
  // ROS Setup
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
}
 
void loop() {
   
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    
    get_velocity();

    rightPubRPM.publish( &rightRPM);
    leftPubRPM.publish( &leftRPM);

    nh.spinOnce();
  }
}

