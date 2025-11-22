#include <PID_v1_bc.h>

void setPWM_30Hz() {
  // Fast PWM 8-bit, prescaler = 1024
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
}

// =============================
// CONFIGURATION
// =============================
const int FAN_PIN = 9;        // PWM pin to MOSFET
const unsigned long POLL_INTERVAL_MS = 250;  // Time between GET commands
const unsigned long SUPERVISOR_TIMEOUT_MS = 2000; // max time without Pi response

unsigned long lastPoll = 0;
unsigned long lastSupervisorResponse = 0;

// =============================
// VARIABLES UPDATED FROM PI
// =============================
double actualTemp = 0.0;   // From Pi
double setpoint = 0.0;     // From Pi
double output = 0;         // PID output
double kp = 2.0, ki = 0.5, kd = 0.0;
int machineState = 0;      // From Pi
int outMin = 0;
int outMax = 255;

// =============================
// PID
// =============================
PID pitController(&actualTemp, &output, &setpoint, kp, ki, kd, DIRECT);

// =============================
// SERIAL BUFFER
// =============================
String incomingLine = "";

// Machine states
enum {
    STATE_IDLE = 0,
    STATE_HEATING = 1,
    STATE_MAINTAINING = 2,
    STATE_COOLING = 3,
    STATE_SHUTDOWN = 4,
    STATE_FAULT = 5
};

// =============================
// SETUP
// =============================
void setup() {
    Serial.begin(115200);
    pinMode(FAN_PIN, OUTPUT);

    pitController.SetMode(AUTOMATIC);
    pitController.SetOutputLimits(outMin, outMax);
}

// =============================
// MAIN LOOP
// =============================
void loop() {

    unsigned long now = millis();

    // Poll supervisor
    if (now - lastPoll >= POLL_INTERVAL_MS) {
        Serial.println("GET");
        lastPoll = now;
    }

    readSupervisorData();

    // Failsafe: stop fan if Pi has not responded recently
    if (now - lastSupervisorResponse > SUPERVISOR_TIMEOUT_MS) {
        analogWrite(FAN_PIN, 0);
        return;
    }

    applyMachineState();

    pitController.Compute();
    analogWrite(FAN_PIN, (int)output);
}

// =============================
// READ SUPERVISOR RESPONSE
// =============================
void readSupervisorData() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            processLine(incomingLine);
            incomingLine = "";
        } else {
            incomingLine += c;
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
    if (eq == -1) return;

    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);

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

// =============================
// APPLY MACHINE STATE
// =============================
void applyMachineState() {
    switch (machineState) {
        case STATE_IDLE:
            output = 0;
            break;

        case STATE_HEATING:
        case STATE_MAINTAINING:
        case STATE_COOLING:
            pitController.SetMode(AUTOMATIC);
            break;

        case STATE_SHUTDOWN:
            output = 255; // full fan purge
            break;

        case STATE_FAULT:
            output = 0;   // immediately stop fan
            break;
    }
}