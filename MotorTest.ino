#define STEP_PIN 4
#define DIR_PIN 5
#define ENABLE_PIN 6

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW); // Enable the driver
  digitalWrite(DIR_PIN, HIGH);   // Set direction
}

void loop() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(500); // Adjust for speed
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(500);
}
