#include <Encoder.h>

// Define encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Create encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test Initialized");

  // Enable internal pull-up resistors for encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
}

void loop() {
  long position = myEncoder.read();
  Serial.print("Encoder Position: ");
  Serial.println(position);
  delay(100);
}
