// Define Pins
const int ENA = 13; // Speed control (PWM capable)
const int IN1 = 14; // Direction pin 1
const int IN2 = 15; // Direction pin 2

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Set motor to maximum speed (255) immediately
  analogWrite(ENA, 255);
}

void loop() {
  // Rapidly switch to FORWARD
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(150); // Move forward for 150 milliseconds

  // Rapidly switch to BACKWARD
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(150); // Move backward for 150 milliseconds
}
