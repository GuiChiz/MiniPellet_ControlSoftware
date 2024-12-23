#include <Wire.h>
#include <Adafruit_MAX31855.h>

// Pin definitions
const int thermoDO1 = 4, thermoCS1 = 5, thermoCLK1 = 6;
const int thermoDO2 = 7, thermoCS2 = 8, thermoCLK2 = 9;
const int thermoDO3 = 10, thermoCS3 = 11, thermoCLK3 = 12;
const int heaterPin1 = 3, heaterPin2 = 4, heaterPin3 = 5;
const int motorPWMPin = 6;

// Thermocouple objects
Adafruit_MAX31855 thermocouple1(thermoCLK1, thermoCS1, thermoDO1);
Adafruit_MAX31855 thermocouple2(thermoCLK2, thermoCS2, thermoDO2);
Adafruit_MAX31855 thermocouple3(thermoCLK3, thermoCS3, thermoDO3);

// PID parameters and variables
// Temperature 1
double setpoint1 = 0, input1 = 0, output1 = 0;
double kp1 = 2.0, ki1 = 5.0, kd1 = 1.0;
double integral1 = 0, previousError1 = 0;

// Temperature 2
double setpoint2 = 0, input2 = 0, output2 = 0;
double kp2 = 2.0, ki2 = 5.0, kd2 = 1.0;
double integral2 = 0, previousError2 = 0;

// Temperature 3
double setpoint3 = 0, input3 = 0, output3 = 0;
double kp3 = 2.0, ki3 = 5.0, kd3 = 1.0;
double integral3 = 0, previousError3 = 0;

// Motor speed
double motorSetpoint = 0, motorInput = 0, motorOutput = 0;
double motorKp = 1.0, motorKi = 0.1, motorKd = 0.05;
double motorIntegral = 0, motorPreviousError = 0;

void setup() {
  Serial.begin(115200);

  // Configure heater pins
  pinMode(heaterPin1, OUTPUT);
  pinMode(heaterPin2, OUTPUT);
  pinMode(heaterPin3, OUTPUT);
  pinMode(motorPWMPin, OUTPUT);
}

void loop() {
  // Read incoming data from the PC
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    parseIncomingData(data);
  }

  // Read temperatures from thermocouples
  input1 = thermocouple1.readCelsius();
  input2 = thermocouple2.readCelsius();
  input3 = thermocouple3.readCelsius();

  // Compute PID for heaters
  output1 = computePID(setpoint1, input1, kp1, ki1, kd1, integral1, previousError1);
  output2 = computePID(setpoint2, input2, kp2, ki2, kd2, integral2, previousError2);
  output3 = computePID(setpoint3, input3, kp3, ki3, kd3, integral3, previousError3);

  // Apply PWM signals to heaters
  analogWrite(heaterPin1, constrain((int)output1, 0, 255));
  analogWrite(heaterPin2, constrain((int)output2, 0, 255));
  analogWrite(heaterPin3, constrain((int)output3, 0, 255));

  // Compute PID for motor
  motorInput = motorSetpoint; // Assume motor responds linearly for simplicity
  motorOutput = computePID(motorSetpoint, motorInput, motorKp, motorKi, motorKd, motorIntegral, motorPreviousError);

  // Apply PWM signal to motor driver
  analogWrite(motorPWMPin, constrain((int)motorOutput, 0, 255));

  // Optional: Send feedback to PC
  sendFeedbackToPC();

  delay(100); // Loop delay
}

double computePID(double setpoint, double input, double kp, double ki, double kd, double &integral, double &previousError) {
  double error = setpoint - input;
  integral += error;
  double derivative = error - previousError;
  previousError = error;
  return (kp * error) + (ki * integral) + (kd * derivative);
}

void parseIncomingData(String data) {
  // Parse the incoming data for setpoints
  int idx1 = data.indexOf(',');
  int idx2 = data.indexOf(',', idx1 + 1);
  int idx3 = data.indexOf(',', idx2 + 1);

  if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
    setpoint1 = data.substring(0, idx1).toDouble();
    setpoint2 = data.substring(idx1 + 1, idx2).toDouble();
    setpoint3 = data.substring(idx2 + 1, idx3).toDouble();
    motorSetpoint = data.substring(idx3 + 1).toDouble();
  }
}

void sendFeedbackToPC() {
  Serial.print("T1:");
  Serial.print(input1);
  Serial.print(", T2:");
  Serial.print(input2);
  Serial.print(", T3:");
  Serial.print(input3);
  Serial.print(", MotorSpeed:");
  Serial.println(motorInput);
}

