// Using MPU9250 library to get angle data from the IMU
#include "MPU9250.h"

// Creating instance of the class MPU9250
MPU9250 mpu;

// Defining constants for motor 1 (Pitch)
#define Encoder1_output_A 3 // pin3 of the Arduino
#define Encoder1_output_B 2 // pin2 of the Arduino
#define Motor1_input_A 4 // pin4 of the Arduino
#define Motor1_input_B 5 // pin5 of the Arduino
#define PWM_input_1 6 // pin6 of the Arduino

// Defining constant for motor 2 (ROLL)
#define Motor2_input_A 7 // pin7 of the Arduino
#define Motor2_input_B 8 // pin8 of the Arduino
#define PWM_input_2 9 // pin9 of the Arduino

// Calibration of pitch motor variables


 
// IMU variables
float roll, pitch, yaw, pitch_angle, roll_angle;

// Pitch PID parameters
float kp_pitch = 1.90, ki_pitch = 2.00, kd_pitch = 0.01, ki_pitch_error = 0, kd_pitch_error = 0, pitch_error = 0, pitch_prev_error = 0, pitch_out = 0, pitch_output = 0;

// Pitch target parameters
int pitch_target = 0, pitch_count_pulses;

// Roll PID parameters
float kp_roll = 1.35, ki_roll = 0.4, kd_roll = 0.01, ki_roll_error = 0, kd_roll_error = 0, roll_error = 0, roll_prev_error = 0, roll_out = 0, roll_output = 0;

// Roll target parameters
int roll_target = 0, roll_count_pulses;

// Motor static friction speed
int pitch_motor_static = 50, roll_motor_static = 28;

// Delta time calculation parameters
float time = 0, prev_time = 0, delta = 0, angle = 0;


/* Setup function */
void setup() {
  
  // Setting up serial connection
  Serial.begin(9600);
  
  // Setting up wire for IMU 
  Wire.begin();
  delay(2000);

  // Checking if the connection is proper
  if (!mpu.setup(0x68)) {  // change to your own address
    while(1){
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // Defining pinMode of motor 1
  pinMode(Encoder1_output_A,INPUT);
  pinMode(Encoder1_output_B,INPUT); 
  pinMode(Motor1_input_B, OUTPUT);
  pinMode(Motor1_input_A, OUTPUT);
  pinMode(PWM_input_1, OUTPUT);

  // Defining pinMode of motor 2
  pinMode(Motor2_input_B, OUTPUT);
  pinMode(Motor2_input_A, OUTPUT);
  pinMode(PWM_input_2, OUTPUT);

//   Calibration of pitch motor to position at 0 level
//  pitch_motor_calibrate();
   
  // Attching Interrupt for motor 1 encoder
  attachInterrupt(digitalPinToInterrupt(Encoder1_output_A),DC_Motor_Encoder,RISING);
  delay(5000);
}


/* Loop function */
void loop() {

  // Checking if there is any changes in IMU and updated it by calling update_roll_pitch_yaw function
  if (mpu.update()) {
    update_roll_pitch();
  }
        
  // Calculating Delta time
  time = micros();
  delta = (time - prev_time)/1.0e6;
  prev_time = time;

  
  // Calling PID controller of roll and pitch
    if (pitch > -70 && pitch < 30)
    { pidpitch();
      pidroll();
    }
    else{
      digitalWrite(PWM_input_1, LOW);
      digitalWrite(PWM_input_2, LOW);
      }
}


/* Function to count encoder pulses */
void DC_Motor_Encoder(){
  int b = digitalRead(Encoder1_output_B);
  if(b > 0){
    pitch_count_pulses++;
  }
  else{
    pitch_count_pulses--;
  }
}


///* Function to calibrate pitch motor */
//void pitch_motor_calibrate(){
//    analogWrite(PWM_input_1, pitch_motor_static);
//    digitalWrite(Motor1_input_A, HIGH);
//    digitalWrite(Motor1_input_B, LOW);
//    delay()
//}



/* Function to update roll and pitch */
void update_roll_pitch() {
  roll = mpu.getRoll();
  pitch = mpu.getPitch();
}


/* Defining PID controller function for pitch */
void pidpitch() {

  // Conversion from angle to count
  pitch_target = (700.00/360.00) * (pitch);
//  Serial.println(pitch);
  
  //  Error calculation
  pitch_error = pitch_target - pitch_count_pulses;

  // Derivative error
  kd_pitch_error = (pitch_error - pitch_prev_error)/ delta;
  
  // Integral error
  if (pitch < 3 && pitch > -3) {
  ki_pitch_error += pitch_error*delta;
  }
  // Total error
  pitch_out = (pitch_error*kp_pitch) + (kd_pitch_error*kd_pitch) + (ki_pitch_error*ki_pitch);

  // Updating previous error
  pitch_prev_error = pitch_error;

  // Limiting the output to 255
  if (pitch_out >= 255){
    pitch_output = 255;
  }
  else if (pitch_out <= -255) {
    pitch_output = 255;
  }
  else if (pitch_out >= 0){
    pitch_output = pitch_out;
  }
  else {
    pitch_output = -pitch_out;
  }

  // Mapping the speed to static friction speed of motor
  if( pitch_output < 3) {
    pitch_output = 0;
  }
  else {
    pitch_output = map(pitch_output, 3.00, 255.00, pitch_motor_static, 255);
  }
  
  // checking the sign of out and assigning the direction to rotate 
  if(pitch_out <= 0){
    digitalWrite(Motor1_input_B, HIGH);
    digitalWrite(Motor1_input_A, LOW);
    analogWrite(PWM_input_1, pitch_output);
  }
  else {
    digitalWrite(Motor1_input_A, HIGH);
    digitalWrite(Motor1_input_B, LOW);
    analogWrite(PWM_input_1, pitch_output);
    }
//   Serial.println(pitch_count_pulses/700.00*360.00); 
  }

/* Defining PID controller function for roll */
void pidroll() {

//  // Conversion from angle to count
//  roll_target = (700.00/360.00) * angle;
  
  //  Error calculation
  roll_error = roll;

  // Derivative error
  kd_roll_error = (roll_error - roll_prev_error)/ delta;
  
  // Integral error
  if (roll > 3 && roll < -3){
  ki_roll_error += roll_error*delta;
  }
  // Total error
  roll_out = (roll_error*kp_roll) + (kd_roll_error*kd_roll) + (ki_roll_error*ki_roll);

  // Updating previous error
  roll_prev_error = roll_error;

  // Limiting the output to 255
  if (roll_out >= 255){
    roll_output = 255;
  }
  else if (roll_out <= -255) {
    roll_output = 255;
  }
  else if (roll_out >= 0){
    roll_output = roll_out;
  }
  else {
    roll_output = -roll_out;
  }

  // Mapping the speed to static friction speed of motor
  if( roll_output < 3) {
    roll_output = 0;
  }
  else {
    roll_output = map(roll_output, 3.00, 255.00, roll_motor_static, 255);
  }
  
  // checking the sign of out and assigning the direction to rotate 
  if(roll_out <= 0){
    digitalWrite(Motor2_input_B, HIGH);
    digitalWrite(Motor2_input_A, LOW);
    analogWrite(PWM_input_2, roll_output);
  }
  else {
    digitalWrite(Motor2_input_A, HIGH);
    digitalWrite(Motor2_input_B, LOW);
    analogWrite(PWM_input_2, roll_output);
    }
  }
