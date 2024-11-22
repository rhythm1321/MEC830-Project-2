#include <Encoder.h>
#include <PID_v1.h>

// Encoder Pins
const int encoderA = 2;
const int encoderB = 3;

// Motor Pins
const int step = 4;
const int dir = 5; 
const int enable = 6;

// Estop 
const int Estop = 8;

bool system_running = true;

void setup() {
Serial.begin(9600);

// Initialize encoder to reference of 0
Enc.write(0);

pinMode(step, OUTPUT);
pinMode(dir, OUTPUT);
pinMode(enable, OUTPUT);
digitalWrite(enable, HIGH); // Initially disable the motor

// Initialize Emergency Stop
pinMode(Estop, INPUT_PULLUP);

}

void loop() {

int Estop_status = digitalRead(Estop);

if (Estop_status == LOW)) {
  system_running = false;
  disableMotor();
  Serial.println("Estop is activated the system is off");
  delay(500);
  return;
}

}
