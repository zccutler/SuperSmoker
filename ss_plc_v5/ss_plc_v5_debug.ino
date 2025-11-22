#include <PID_v1_bc.h>

// =============================
// PWM @ 30 Hz (retain original behavior)
// =============================
void setPWM_30Hz() {
  // Fast PWM 8-bit, prescaler = 1024
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
}

// =============================
// CONFIGURATION
// =============================
const int FAN_PIN = 9;
const unsigned long POLL_INTERVAL_MS = 250;      // used as a minimum interval between GETs when not waiting
const unsigned long SUPERVISOR_TIMEOUT_MS = 2000;
const unsigned long PID_INTERVAL_MS = 100;       // 10 Hz PID loop

unsigned long lastPoll = 0;
unsigned long lastGetTime = 0;
unsigned long lastSupervisorResponse = 0;
unsigned long lastPID = 0;

// =============================
// VARIABLES FROM SUPERVISOR
// =============================
double actualTemp = 0.0;
double setpoint = 0.0;
double output = 0;
double kp = 2.0, ki = 0.5, kd = 0.0;
int machineState = 0;   // 0 = IDLE, 1 = ACTIVE
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

// NEW simplified states
enum {
    STATE_IDLE = 0,
    STATE_ACTIVE = 1
};

// New handshake control
bool waitingForSupervisor = false;

// =============================
// SETUP
// =============================
void setup() {
    Serial.begin(115200);
    pinMode(FAN_PIN, OUTPUT);

    setPWM_30Hz();

    pitController.SetMode(AUTOMATIC);
    pitController.SetOutputLimits(outMin, outMax);
    pitController.SetSampleTime(PID_INTERVAL_MS);  // force 10 Hz internal timing
}

// =============================
// MAIN LOOP
// =============================
void loop() {
    unsigned long now = millis();

    // Send GET only when not waiting for supervisor OR if never polled yet and POLL_INTERVAL elapsed
    if (!waitingForSupervisor && (now - lastPoll >= POLL_INTERVAL_MS)) {
        Serial.println("GET");
        lastPoll = now;
        lastGetTime = now;
        waitingForSupervisor = true;
    }

    // Read any incoming supervisor data
    readSupervisorData();

    // If we're waiting but supervisor hasn't responded within the timeout, give up and clear waiting flag
    if (waitingForSupervisor && (now - lastGetTime > SUPERVISOR_TIMEOUT_MS)) {
        // timed out waiting for a full response -> go to failsafe
        waitingForSupervisor = false;
        // Do not update lastSupervisorResponse here; leave it for valid END receipt
        output = 0;
        // Optional debug:
        // Serial.println("DBG: SUP timeout, clearing waitingForSupervisor and forcing output=0");
    }

    // Default: zero output if failsafe triggered or IDLE
    if (machineState == STATE_IDLE || now - lastSupervisorResponse > SUPERVISOR_TIMEOUT_MS) {
        output = 0;
    } 
    // ACTIVE state → run PID at 10 Hz
    else if (now - lastPID >= PID_INTERVAL_MS) {
        pitController.Compute();
        lastPID = now;
    }

    // Always write PWM, even if output hasn't changed
    analogWrite(FAN_PIN, (int)output);
}

// =============================
// READ SUPERVISOR RESPONSE
// =============================
void readSupervisorData() {
    while (Serial.available()) {
        char c = Serial.read();

        // Accept CR and LF as line terminators; build line until '\n' seen
        if (c == '\n') {
            processLine(incomingLine);
            incomingLine = "";
        } else if (c != '\r') {
            // ignore '\r' characters, append other characters
            incomingLine += c;
            // guard against runaway String growth
            if (incomingLine.length() > 512) {
                incomingLine = "";
            }
        }
    }
}

void processLine(String line) {
    line.trim();

    // If supervisor sends END (exact token), we consider the response complete.
    if (line == "END") {
        lastSupervisorResponse = millis();
        // Successful full response received, allow next GET
        waitingForSupervisor = false;
        return;
    }

    int eq = line.indexOf('=');
    if (eq == -1) {
        // If you want to debug ignored or malformed lines, uncomment:
        // Serial.print("IGNORED: "); Serial.println(line);
        return;
    }

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
