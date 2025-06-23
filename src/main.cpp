#include <Wire.h>
#include <Adafruit_MCP4725.h>

//================================================================
// MOTOR CONTROL SETUP
//================================================================
Adafruit_MCP4725 dacRight;
Adafruit_MCP4725 dacLeft;

#define DIR_LEFT 7
#define DIR_RIGHT 6
#define DAC_RESOLUTION 4096
#define WHEEL_RADIUS 0.08255
#define WHEEL_BASE   0.47

//================================================================
// SPEED READER SETUP
//================================================================
const int SPEED_PULSE_PIN_LEFT = 2;
const int SPEED_PULSE_PIN_RIGHT = 3;
int motorPolePairs = 15;

volatile unsigned long pulseCountLeft = 0;
volatile unsigned long pulseCountRight = 0;
float measuredRpmLeft = 0.0;
float measuredRpmRight = 0.0;

unsigned long previousRpmCalcTime = 0;
const long rpmCalcInterval = 300;

// Filtering parameters
float lastPublishedRpmLeft = 0.0;
float lastPublishedRpmRight = 0.0;
const float spikeThreshold = 310.0; // max allowed jump per 250ms sample

unsigned long previousPublishTime = 0;
const long rpmPublishInterval = 100;  // Publish every 100 ms

//================================================================
// ISR
//================================================================
void countPulseLeftISR() {
  pulseCountLeft++;
}
void countPulseRightISR() {
  pulseCountRight++;
}

//================================================================
// MOTOR CONTROL
//================================================================
int rpmToBitsRight(float rpm) {
  return (int)((rpm - 2.43) / 0.2087);
}
int rpmToBitsLeft(float rpm) {
  return (int)((rpm - 4.1) / 0.2014);
}
void setLeftMotorRPM(float rpm) {
  digitalWrite(DIR_LEFT, rpm >= 0 ? HIGH : LOW);
  rpm = fabs(rpm);
  int bits = constrain(rpmToBitsLeft(rpm), 0, DAC_RESOLUTION - 1);
  dacLeft.setVoltage(bits, false);
}
void setRightMotorRPM(float rpm) {
  digitalWrite(DIR_RIGHT, rpm >= 0 ? LOW : HIGH);
  rpm = fabs(rpm);
  int bits = constrain(rpmToBitsRight(rpm), 0, DAC_RESOLUTION - 1);
  dacRight.setVoltage(bits, false);
}
float velocityToRPM(float v) {
  return (v / (2 * PI * WHEEL_RADIUS)) * 60.0;
}

//================================================================
// SETUP
//================================================================
void setup() {
  Serial.begin(115200);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  dacRight.begin(0x60);
  dacLeft.begin(0x61);
  setLeftMotorRPM(0);
  setRightMotorRPM(0);

  pinMode(SPEED_PULSE_PIN_LEFT, INPUT_PULLUP);
  pinMode(SPEED_PULSE_PIN_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_PULSE_PIN_LEFT), countPulseLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(SPEED_PULSE_PIN_RIGHT), countPulseRightISR, RISING);
}

//================================================================
// LOOP
//================================================================
void loop() {
  static uint8_t buf[12];
  static uint8_t idx = 0;
  while (Serial.available() > 0 && idx < 12) {
    buf[idx++] = Serial.read();
  }

  if (idx == 12) {
    float x, vy, omega;
    memcpy(&x,     buf,     4);
    memcpy(&vy,    buf + 4, 4);
    memcpy(&omega, buf + 8, 4);

    float v_left  = x - (omega * WHEEL_BASE / 2.0);
    float v_right = x + (omega * WHEEL_BASE / 2.0);
    float rpm_left_cmd  = velocityToRPM(v_left);
    float rpm_right_cmd = velocityToRPM(v_right);

    setLeftMotorRPM(rpm_left_cmd);
    setRightMotorRPM(rpm_right_cmd);
    idx = 0;
  }

  unsigned long currentMillis = millis();

  // Recalculate RPM every rpmCalcInterval (e.g. 300ms)
  if (currentMillis - previousRpmCalcTime >= rpmCalcInterval) {
    float intervalSeconds = (currentMillis - previousRpmCalcTime) / 1000.0;

    noInterrupts();
    unsigned long capturedPulseLeft = pulseCountLeft;
    unsigned long capturedPulseRight = pulseCountRight;
    pulseCountLeft = 0;
    pulseCountRight = 0;
    interrupts();

    previousRpmCalcTime = currentMillis;

    float freqLeft = capturedPulseLeft / intervalSeconds;
    float freqRight = capturedPulseRight / intervalSeconds;
    float rawRpmLeft = (freqLeft / motorPolePairs) * 10.0;
    float rawRpmRight = (freqRight / motorPolePairs) * 10.0;

    measuredRpmLeft = (digitalRead(DIR_LEFT) == HIGH) ? rawRpmLeft : -rawRpmLeft;
    measuredRpmRight = (digitalRead(DIR_RIGHT) == LOW) ? rawRpmRight : -rawRpmRight;
  }

  // Publish the last measured RPM more frequently
  if (currentMillis - previousPublishTime >= rpmPublishInterval) {
    float publishRpmLeft = measuredRpmLeft;
    float publishRpmRight = measuredRpmRight;

    if (fabs(publishRpmLeft - lastPublishedRpmLeft) > spikeThreshold) {
      publishRpmLeft = lastPublishedRpmLeft;
    }
    if (fabs(publishRpmRight - lastPublishedRpmRight) > spikeThreshold) {
      publishRpmRight = lastPublishedRpmRight;
    }

    float rpm_data[2] = {publishRpmLeft, publishRpmRight};
    Serial.write((uint8_t*)rpm_data, sizeof(rpm_data));

    lastPublishedRpmLeft = publishRpmLeft;
    lastPublishedRpmRight = publishRpmRight;
    previousPublishTime = currentMillis;
  }
}