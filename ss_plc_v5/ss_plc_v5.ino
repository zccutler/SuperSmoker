#include <PID_v1_bc.h>

// =============================
// CONFIGURATION
// =============================
const int FAN_PIN = 9;
const unsigned long SUPERVISOR_TIMEOUT_MS = 2000; // 2 sec timeout
const unsigned long GET_INTERVAL_MS = 1000;       // min interval between GETs

// Supervisor state tracking
unsigned long lastSupervisorResponse = 0;
unsigned long lastGetSent = 0;
bool waitingForResponse = false;

// Supervisor-controlled variables
double actualTemp = 0.0;
double setpoint = 0.0;
double kp = 2.0, ki = 0.5, kd = 0.0;
int machineState = 0;   // 0 = IDLE, 1 = ACTIVE
int outMin = 0, outMax = 255;

// PID
double output = 0;
PID pitController(&actualTemp, &output, &setpoint, kp, ki, kd, DIRECT);

// Serial parsing
String incomingLine = "";

// =============================
// HELPER FUNCTIONS
// =============================
void setPWM_30Hz() {
    // Fast PWM 8-bit, prescaler = 1024
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
}

void sendGET() {
    Serial.println("GET");
    waitingForResponse = true;
    lastGetSent = millis();
}

void processLine(String line) {
    line.trim();
    if (line == "END") {
        lastSupervisorResponse = millis();
        waitingForResponse = false;
        return;
    }

    int eq = line.indexOf('=');
    if (eq < 0) return;

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

// =============================
// SETUP
// =============================
void setup() {
    Serial.begin(115200);
    pinMode(FAN_PIN, OUTPUT);
    setPWM_30Hz();

    pitController.SetMode(AUTOMATIC);
    pitController.SetOutputLimits(outMin, outMax);
}

// =============================
// MAIN LOOP
// =============================
void loop() {
    unsigned long now = millis();

    // Process any supervisor input
    readSupervisorData();

    // Check if timeout has occurred
    if (waitingForResponse && (now - lastGetSent > SUPERVISOR_TIMEOUT_MS)) {
        waitingForResponse = false;
        machineState = 0; // fallback to idle
    }

    // Send GET if allowed (conversation idle and interval elapsed)
    if (!waitingForResponse && (now - lastGetSent >= GET_INTERVAL_MS)) {
        sendGET();
    }

    // Compute PID if machine active
    if (machineState == 1) pitController.Compute();
    else output = 0;

    // Update PWM regardless
    analogWrite(FAN_PIN, (int)output);
}
