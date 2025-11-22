/*********************************************************************
 * Supersmoker PLC (UNO) Firmware – PID at Fixed Rate
 * ---------------------------------------------------
 * - PID evaluates at constant interval (PID_DT_MS)
 * - Supervisor provides:
 *      SETPOINT
 *      MODE = IDLE or ACTIVE
 * - Lid-open logic handled by supervisor by switching MODE
 * - Integral frozen during MODE == IDLE
 * - Output scaled to 0–255 for PWM fan control
 *********************************************************************/

#include <Arduino.h>

/* --------------------------
   USER CONFIGURATIONS
   -------------------------- */
const int FAN_PIN = 9;          // PWM fan pin
const unsigned long PID_DT_MS = 100;   // PID frequency: 100ms = 10 Hz

/* --------------------------
   PID PARAMETERS
   (Supervisor will send these – but we declare defaults)
   -------------------------- */
float Kp = 4.0;
float Ki = 0.10;
float Kd = 1.0;

/* --------------------------
   RUNTIME STATE
   -------------------------- */
float setpoint = 0.0;
float currentTemp = 0.0;        // received via GET response
String mode = "IDLE";           // from supervisor

// Internal PID memory
float integral = 0.0;
float lastError = 0.0;
unsigned long lastPID = 0;

/* --------------------------
   Helper: Clamp float
   -------------------------- */
float clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

/* --------------------------
   PROCESS SUPERVISOR RESPONSE
   -------------------------- */
void processSupervisorLine(const String &line) {
  if (line.startsWith("SETPOINT=")) {
    setpoint = line.substring(9).toFloat();
  }
  else if (line.startsWith("MODE=")) {
    mode = line.substring(5);
    mode.trim();

    if (mode == "IDLE") {
      // freeze integral and force output to zero
      integral = 0;
      analogWrite(FAN_PIN, 0);
    }
  }
  else if (line.startsWith("TEMP=")) {
    // optional: supervisor may send temp back for consistency
    currentTemp = line.substring(5).toFloat();
  }
}

/* --------------------------
   SEND GET REQUEST
   -------------------------- */
void requestUpdate() {
  Serial.println("GET");
  Serial.flush();
}

/* --------------------------
   PID EVALUATION
   -------------------------- */
void runPID() {
  if (mode == "IDLE") {
    // No PID activity, no output
    analogWrite(FAN_PIN, 0);
    return;
  }

  float error = setpoint - currentTemp;

  // Integral with anti-windup
  integral += error * (PID_DT_MS / 1000.0);

  // Clamp integral to prevent runaway
  integral = clamp(integral, -500.0, 500.0);

  float derivative = (error - lastError) / (PID_DT_MS / 1000.0);

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Scale to 0–255 for PWM output
  int pwm = (int)clamp(output, 0.0, 255.0);

  analogWrite(FAN_PIN, pwm);

  lastError = error;
}

/* --------------------------
   SETUP
   -------------------------- */
void setup() {
  Serial.begin(115200);
  pinMode(FAN_PIN, OUTPUT);
  analogWrite(FAN_PIN, 0);
  delay(500);
}

/* --------------------------
   LOOP
   -------------------------- */
void loop() {

  /* 1. Handle incoming lines from the supervisor */
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      processSupervisorLine(line);
    }
  }

  /* 2. Run PID at fixed interval */
  unsigned long now = millis();
  if (now - lastPID >= PID_DT_MS) {
    lastPID = now;
    runPID();
    requestUpdate();    // get fresh readings from supervisor
  }
}
