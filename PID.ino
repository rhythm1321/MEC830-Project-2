#include <Encoder.h>
#include <AccelStepper.h>
#include <PID_v1.h>

// Encoder Pins
const int encoderA = 2;
const int encoderB = 3;

// Motor Pins
const int step = 4;
const int dir = 5; 
const int enable = 6;

// Switches
const int toggle = 7; // On/Off switch
const int Estop = 8; // Emergency Stop

// Motor driver pins for microstepping
const int M0_PIN = 9;     
const int M1_PIN = 10;     
const int M2_PIN = 11;    

// Stepper Configuration
AccelStepper stepper(AccelStepper::DRIVER, step, dir);

// Define PID Parameters
double P_angle = 0.0;  // Current pendulum angle
double P_set = 0.0;    // Target angle (setpoint)
double PosOut = 0.0;     // PID output for motor control
double Kp = 250.00, Ki = 0.00, Kd = 5.00; // PID gains

// Encoder and PID Objects
Encoder Enc(encoderA, encoderB); // Initialize encoder
PID sysPID(&P_angle, &PosOut, &P_set, Kp, Ki, Kd, DIRECT);

// Motion parameters for safety
const double p_angle_limit = 45;
const int max_steps = 500; // Fixed constant definition
bool system_running = true;

// Microstepping configuration
struct MicrostepConfig {
  int steps_per_mm;
  int microresolution;
  bool M0;
  bool M1;
  bool M2;
};

// Array of microstepping configurations
MicrostepConfig microstepConfigs[] = {
  {5, 1, LOW, LOW, LOW},    // Full step
  {10, 2, HIGH, LOW, LOW},  // Half step
  {20, 4, LOW, HIGH, LOW},  // Quarter step
  {40, 8, HIGH, HIGH, LOW}, // Eighth step
  {80, 16, LOW, LOW, HIGH}, // Sixteenth step
  {160, 32, HIGH, HIGH, HIGH} // Thirty-second step
};

// Select the desired microstepping mode
const int stepMode = 3; 
double steps_per_mm;

void setup() {

  Serial.begin(9600);

  // Initialize encoder to reference of 0
  Enc.write(0);

  // Motor
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(enable, HIGH); // Initially disable the motor

  // Motor microstepping
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  MicrostepConfig config = microstepConfigs[stepMode];
  digitalWrite(M0_PIN, config.M0);
  digitalWrite(M1_PIN, config.M1);
  digitalWrite(M2_PIN, config.M2);
  steps_per_mm = config.steps_per_mm;

  // Initialize Switches
  pinMode(toggle, INPUT_PULLUP); 
  pinMode(Estop, INPUT_PULLUP);

  // Configure Stepper Motor
  stepper.setMaxSpeed(5000000);
  stepper.setMinPulseWidth(5);

  sysPID.SetMode(AUTOMATIC);
  sysPID.SetOutputLimits(-max_steps, max_steps); // limiting the PID output to the set motor step range
}

void loop() {

  int Estop_status = digitalRead(Estop);

  if (Estop_status == LOW) {
    system_running = false;
    disableMotor();
    Serial.println("E-Stop is activated. The system is OFF.");
    delay(500);
    return;
  }

  if (Estop_status == HIGH) {
    system_running = true;
    Serial.println("System has been reset. Resuming movement now...");
    return;
  }

  updateEncoderAngle();
  if (abs(P_angle) > p_angle_limit) {
    system_running = false; 
    disableMotor();
    Serial.println("System is OFF due to angle exceeding limit.");
    return;
  }

  // PID Computing
  sysPID.Compute();
  driveMotor(PosOut); // Drive the motor based on the PID output

  Serial.print("Angle: ");
  Serial.print(P_angle);
  Serial.print(" | PID Output: ");
  Serial.println(PosOut);

  delay(20);
}

void updateEncoderAngle() {
  long count = Enc.read();
  P_angle = count * (360.00 / 2000.00); // Convert encoder counts to degrees
}

void driveMotor(double pidOutput) {
  if (abs(P_angle) < p_angle_limit) {
    stepper.setSpeed(pidOutput);
    stepper.runSpeed();
  } else {
    disableMotor();
  }
}

void disableMotor() {
  digitalWrite(enable, HIGH); 
  stepper.stop(); 
}
