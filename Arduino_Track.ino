#include <Wire.h>
#include <Servo.h>

// Scanning state variables
enum ScanState {
  scan_right,
  scan_left,
  return_center
};
enum ScanState current_scan_state = scan_right;

float PID(int ballCenterX);
void stopMotors();
void turnLeft(int control_signal);
void turnRight(int control_signal);
void driveForward();
void runMotorForDuration(void (*motorFunction)(int), int control_signal, int interval);
void servoScan(int *angle, ScanState *current_scan_state, int change_angle, int sign_to_search);
void servoReturn(int *angle, ScanState *current_scan_state, int control_signal, int interval);
long getUltrasonicDistance();

// Motor Pins
const int ena = 3; // Left motor
const int enb = 13; // Right motor

const int in1 = 2;  // Left motor forward
const int in2 = 4;  // Left motor backward
const int in3 = 9;  // Right motor forward
const int in4 = 10;  // Right motor backward

const int default_speed = 180;  // 0 to 255
float threshold = 4.5;

// Control parameters
unsigned long previous_time = 0;  
float previous_error = 0;
float integral = 0;
float control_signal = 0.0;

// Ultrasonic sensor pins
const int trig_pin = 7;
const int echo_pin = 6;

// Servo pin
const int servo_pin = 12;
Servo myServo;

// Servo angle limits for scanning
const int angle_middle = 100;
const int angle_right = 50;
const int angle_left = 150;
int angle = angle_middle;

int sign_to_search = 1;

long front_distance;

unsigned long previous_servo_time = 0; 
const unsigned long servo_interval = 20;

int32_t ball_center_x = 0;
int moving = 0; // Sign we search the center

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Wire.begin();

  // Initialize pins
  myServo.attach(servo_pin);
  myServo.write(angle);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  delay(100);
}

void loop() {
  if (Serial.available() > 0) { 
    String receivedData = Serial.readStringUntil('\n');
    ball_center_x = receivedData.toInt();
    
    front_distance = getUltrasonicDistance(); 
    Serial.print("Obstacle in ");
    Serial.print(front_distance);
    Serial.println(" cm."); 
                
    if (front_distance < 30) {
      stopMotors();
      Serial.println("Finish!");
      return;
    }
    // If the ball isn't detect (-1 means, not see a ball)
    if (ball_center_x == -1) {
      stopMotors();
      // Scan 
      if (current_scan_state != return_center) {
        if (millis() - previous_servo_time >= servo_interval){
          previous_servo_time = millis();
          servoScan(&angle, &current_scan_state, 10, sign_to_search);
        }
      }
    }
  
    // If the ball is detect
    else {
      // After scanning, return back the servo to the middle
      control_signal = PID(ball_center_x);
      if (angle != angle_middle) {
        sign_to_search = 0; // For not searching another time
        if (millis() - previous_servo_time >= servo_interval){
          previous_servo_time = millis();
          servoReturn(&angle, &current_scan_state, control_signal, 300);
        }
      }
  
      // If we didn't scan at all or we return the car to start position, lets apply controller.
      else {
        Serial.println(control_signal);
        if (abs(control_signal) > threshold && !moving){
          // Positive Feedback -> Turn left
          if (control_signal > 0) {
            runMotorForDuration(turnLeft, control_signal, 150);
          }
          else {
            runMotorForDuration(turnRight, abs(control_signal), 150);
          }
        } 
        else {   
          //driveForward(); 
          //moving = 1;
        }
      }
    }
  }
}

float PID(int ballCenterX) {
  float Kp = 0.05;
  float Ki = 0.0;
  float Kd = 0.025;

  unsigned long current_time = millis();
  // Convert milliseconds to seconds
  float delta_time = (float)(current_time - previous_time) / 1000.0;  

  unsigned int setpoint = 320;  // Center of the frame (640/2)

  int error = setpoint - ballCenterX;
  integral += error * delta_time;
  float derivative = (error - previous_error) / delta_time;

  previous_error = error;
  previous_time = current_time;
  return (Kp * error) + (Kd * derivative) + (Ki * integral);
}

void stopMotors(){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(ena, 0);
    analogWrite(enb, 0);
}

void turnRight(int control_signal){
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH);  // Left motor forward
    digitalWrite(in3, LOW);  
    digitalWrite(in4, HIGH); // Right motor backward

    // Adjust PWM duty cycle
    analogWrite(ena, default_speed - control_signal); // Left
    analogWrite(enb, default_speed + control_signal); // Right
}

void turnLeft(int control_signal){
    digitalWrite(in1, HIGH); // Left motor backward
    digitalWrite(in2, LOW); 
    digitalWrite(in3, HIGH); // Right motor forward
    digitalWrite(in4, LOW);  

    analogWrite(ena, default_speed + control_signal);
    analogWrite(enb, default_speed - control_signal);
}

void driveForward(){
    analogWrite(ena, default_speed - 35); // Left side is a little heavier
    analogWrite(enb, default_speed - 50);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
}

void servoScan(int *angle, ScanState *current_scan_state, int change_angle, int sign_to_search){
  if (sign_to_search){
    switch (*current_scan_state) {
      case scan_right:
        *angle -= change_angle;
        if (*angle == angle_right) {
          *current_scan_state = scan_left;
        }
      break;
      
      case scan_left:
        *angle += change_angle;
        if (*angle == angle_left) {
          *current_scan_state = return_center;
          *angle = angle_middle;
        }
      break;
    }
    Serial.println(*angle);
    myServo.write(*angle);
  }
}

void servoReturn(int *angle, ScanState *current_scan_state, float control_signal, int interval){
  *angle = angle_middle;
  switch (*current_scan_state) {
    case scan_right:
      runMotorForDuration(turnRight, control_signal, interval);
    break;
    
    case scan_left:
      runMotorForDuration(turnLeft, control_signal, interval);
    break;
  }
  *current_scan_state = return_center;
  myServo.write(*angle);
}

void runMotorForDuration(void (*motorFunction)(int), int control_signal, int interval) {
  unsigned long startMillis = millis();  
  while (millis() - startMillis <= interval) {
    motorFunction(control_signal);  
  }
  stopMotors(); 
}

long getUltrasonicDistance() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  
  long duration = pulseIn(echo_pin, HIGH, 30000);

  if (duration == 0) {
    return 999;  // No echo detected (timeout)
  }
  
  // Calculate the distance (in centimeters)
  long distance = duration * 0.0343 / 2;
  return distance;
}
