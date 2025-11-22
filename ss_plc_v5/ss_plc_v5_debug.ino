// Minimal debug overlay: echo parsed supervisor lines and current PWM every second.

#include <PID_v1_bc.h>

const int FAN_PIN = 9;
const unsigned long POLL_INTERVAL_MS = 250;
const unsigned long SUPERVISOR_TIMEOUT_MS = 2000;
const unsigned long PID_INTERVAL_MS = 100;

unsigned long lastPoll = 0;
unsigned long lastSupervisorResponse = 0;
unsigned long lastPID = 0;
unsigned long lastDebug = 0;

double actualTemp = 0.0;
double setpoint = 0.0;
double output = 0;
double kp = 2.0, ki = 0.5, kd = 0.0;
int machineState = 0;
int outMin = 0;
int outMax = 255;

PID pitController(&actualTemp, &output, &setpoint, kp, ki, kd, DIRECT);

String incomingLine = "";

enum { STATE_IDLE = 0, STATE_ACTIVE = 1 };

void setPWM_30Hz() {
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
}

void setup() {
  Serial.begin(115200);
  pinMode(FAN_PIN, OUTPUT);
  setPWM_30Hz();
  pitController.SetMode(AUTOMATIC);
  pitController.SetOutputLimits(outMin, outMax);
  pitController.SetSampleTime(PID_INTERVAL_MS);
}

void loop() {
  unsigned long now = millis();
  if (now - lastPoll >= POLL_INTERVAL_MS) {
    Serial.println("GET");
    lastPoll = now;
  }
  readSupervisorData();

  if (machineState == STATE_IDLE || now - lastSupervisorResponse > SUPERVISOR_TIMEOUT_MS) {
    output = 0;
  } else if (now - lastPID >= PID_INTERVAL_MS) {
    pitController.Compute();
    lastPID = now;
  }
  analogWrite(FAN_PIN, (int)output);

  if (now - lastDebug >= 1000) {
    Serial.print("DBG STATE=");
    Serial.print(machineState);
    Serial.print(" OUT=");
    Serial.print((int)output);
    Serial.print(" TEMP=");
    Serial.print(actualTemp);
    Serial.print(" SET=");
    Serial.println(setpoint);
    lastDebug = now;
  }
}

void readSupervisorData() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processLine(incomingLine);
      incomingLine = "";
    } else {
      incomingLine += c;
      // guard to prevent runaway String growth
      if (incomingLine.length() > 512) {
        incomingLine = "";
      }
    }
  }
}

void processLine(String line) {
  line.trim();

  if (line == "END") {
    lastSupervisorResponse = millis();
    return;
  }

  int eq = line.indexOf('=');
  if (eq == -1) {
    Serial.print("IGNORED:");
    Serial.println(line);
    return;
  }

  String key = line.substring(0, eq);
  String val = line.substring(eq + 1);

  Serial.print("PARSE:");
  Serial.print(key);
  Serial.print("=");
  Serial.println(val);

  if (key == "TEMP") actualTemp = val.toFloat();
  else if (key == "SETPOINT") setpoint = val.toFloat();
  else if (key == "STATE") machineState = val.toInt();
  else if (key == "KP") kp = val.toFloat();
  else if (key == "KI") ki = val.toFloat();
  else if (key == "KD") kd = val.toFloat();
  else if (key == "OUTMIN") outMin = val.toInt();
  else if (key == "OUTMAX") outMax = val.toInt();

  pitController.SetTunings(kp, ki, kd);
  pitController.SetOutputLimits(outMin, outMax);
}
