#include <PID_v1.h>
#include <AccelStepper.h>
#include <max6675.h>

// Pin definizione
#define THERMOCOUPLE_1_DO 4
#define THERMOCOUPLE_1_CS 5
#define THERMOCOUPLE_1_CLK 6
#define THERMOCOUPLE_2_DO 7
#define THERMOCOUPLE_2_CS 8
#define THERMOCOUPLE_2_CLK 9
#define THERMOCOUPLE_3_DO 10
#define THERMOCOUPLE_3_CS 11
#define THERMOCOUPLE_3_CLK 12
#define HEATER_1_PIN 3
#define HEATER_2_PIN 5
#define HEATER_3_PIN 6
#define MOTOR_STEP_PIN 2
#define MOTOR_DIR_PIN 3

// Termocoppie
MAX6675 thermocouple1(THERMOCOUPLE_1_CLK, THERMOCOUPLE_1_CS, THERMOCOUPLE_1_DO);
MAX6675 thermocouple2(THERMOCOUPLE_2_CLK, THERMOCOUPLE_2_CS, THERMOCOUPLE_2_DO);
MAX6675 thermocouple3(THERMOCOUPLE_3_CLK, THERMOCOUPLE_3_CS, THERMOCOUPLE_3_DO);

// Variabili di controllo
double setpointTemp1 = 0, setpointTemp2 = 0, setpointTemp3 = 0;
double currentTemp1 = 0, currentTemp2 = 0, currentTemp3 = 0;
double outputPWM1 = 0, outputPWM2 = 0, outputPWM3 = 0;

double setpointSpeed = 0;
double currentSpeed = 0;
double outputSpeedPWM = 0;

// PID Controller
PID pidTemp1(&currentTemp1, &outputPWM1, &setpointTemp1, 2, 5, 1, DIRECT);
PID pidTemp2(&currentTemp2, &outputPWM2, &setpointTemp2, 2, 5, 1, DIRECT);
PID pidTemp3(&currentTemp3, &outputPWM3, &setpointTemp3, 2, 5, 1, DIRECT);
PID pidSpeed(&currentSpeed, &outputSpeedPWM, &setpointSpeed, 1, 0.1, 0.05, DIRECT);

// Motore stepper
AccelStepper stepper(AccelStepper::DRIVER, MOTOR_STEP_PIN, MOTOR_DIR_PIN);

// Intervallo di aggiornamento
unsigned long lastTime = 0;
unsigned long updateInterval = 100; // ms

void setup() {
  // Impostazione seriale
  Serial.begin(9600);

  // Impostazione PID
  pidTemp1.SetMode(AUTOMATIC);
  pidTemp2.SetMode(AUTOMATIC);
  pidTemp3.SetMode(AUTOMATIC);
  pidSpeed.SetMode(AUTOMATIC);

  // Impostazione heater
  pinMode(HEATER_1_PIN, OUTPUT);
  pinMode(HEATER_2_PIN, OUTPUT);
  pinMode(HEATER_3_PIN, OUTPUT);

  // Impostazione stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

void loop() {
  // Lettura setpoint dal PC
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseSetpoints(input);
  }

  // Lettura temperature
  currentTemp1 = thermocouple1.readCelsius();
  currentTemp2 = thermocouple2.readCelsius();
  currentTemp3 = thermocouple3.readCelsius();

  // Esecuzione PID
  pidTemp1.Compute();
  pidTemp2.Compute();
  pidTemp3.Compute();
  pidSpeed.Compute();

  // Applicazione dei risultati PID
  analogWrite(HEATER_1_PIN, constrain(outputPWM1, 0, 255));
  analogWrite(HEATER_2_PIN, constrain(outputPWM2, 0, 255));
  analogWrite(HEATER_3_PIN, constrain(outputPWM3, 0, 255));

  // Controllo motore stepper
  stepper.setSpeed(outputSpeedPWM);
  stepper.runSpeed();

  // Ritardo aggiornamento
  delay(updateInterval);
}

// Parsing dei setpoint
void parseSetpoints(String input) {
  char delimiter = ',';
  int idx1 = input.indexOf(delimiter);
  int idx2 = input.indexOf(delimiter, idx1 + 1);
  int idx3 = input.indexOf(delimiter, idx2 + 1);

  if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
    setpointTemp1 = input.substring(0, idx1).toDouble();
    setpointTemp2 = input.substring(idx1 + 1, idx2).toDouble();
    setpointTemp3 = input.substring(idx2 + 1, idx3).toDouble();
    setpointSpeed = input.substring(idx3 + 1).toDouble();
  }
}
