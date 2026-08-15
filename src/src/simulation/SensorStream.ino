/*
  Real-Time Sensor Event Generator for Robotic Vehicle Navigation
  Board: Arduino Uno / Mega / ESP32
*/

#define PIN_TRIG 9
#define PIN_ECHO 10
#define PIN_IR   A0

unsigned long eventSequence = 1000;
unsigned long lastSensorSweep = 0;
const unsigned long SWEEP_INTERVAL_MS = 150;

int vehicleGridX = 12;
int vehicleGridY = 8;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  randomSeed(analogRead(A5));
  while (!Serial) { ; }
}

long readUltrasonicDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) return 400;
  return duration * 0.034 / 2;
}

double readInfraredDistanceMeters() {
  int rawADC = analogRead(PIN_IR);
  if (rawADC < 10) rawADC = 10;
  double volts = rawADC * (5.0 / 1023.0);
  return (65.0 / (volts + 0.4)) / 100.0;
}

void emitEvent(const char* sensorType, double valMeters, int cellX, int cellY) {
  eventSequence++;
  char tsBuffer[32];
  unsigned long now = millis();
  snprintf(tsBuffer, sizeof(tsBuffer), "2026-08-15T10:%02lu:%02lu.%03luZ",
           (now / 60000) % 60, (now / 1000) % 60, now % 1000);

  Serial.print(F("{\"event_id\":\"EVT_ARD_"));
  Serial.print(eventSequence);
  Serial.print(F("\",\"sensor_type\":\""));
  Serial.print(sensorType);
  Serial.print(F("\",\"timestamp\":\""));
  Serial.print(tsBuffer);
  Serial.print(F("\",\"value\":"));
  Serial.print(valMeters, 2);
  Serial.print(F(",\"location\":{\"x\":"));
  Serial.print(cellX);
  Serial.print(F(",\"y\":"));
  Serial.print(cellY);
  Serial.println(F("}}"));
}

void loop() {
  if (millis() - lastSensorSweep >= SWEEP_INTERVAL_MS) {
    lastSensorSweep = millis();

    // 1. Ultrasonic Reading
    long usDistanceCm = readUltrasonicDistanceCm();
    emitEvent("ULTRASONIC", usDistanceCm / 100.0, vehicleGridX + 1, vehicleGridY);

    // 2. Infrared Reading (targeted at same obstacle coordinate)
    double irDist = readInfraredDistanceMeters();
    emitEvent("INFRARED", irDist, vehicleGridX + 1, vehicleGridY);

    // 3. Simulated 2D LiDAR Reading
    double lidarDist = 1.45 + ((random(-10, 10)) / 100.0);
    emitEvent("LIDAR", lidarDist, vehicleGridX + 1, vehicleGridY);
  }
}
