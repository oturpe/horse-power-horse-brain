// Output pin for indicator led
#define INDICATOR_PIN 13

// Digital input pin for skin readings
#define SKIN_PIN 2

// Digital output pin for motor enable
#define MOTOR_ENABLE_PIN 10
// Digital output pin for motor forward run
#define MOTOR_FORWARD_PIN 9
// Digital output pin for motor reverse run
#define MOTOR_REVERSE_PIN 8

// Smoothing frame length. Given in units of millisecond
#define SKIN_SMOOTHING_FRAME_LENGTH 300

// How many pulses have to be received in a frame to activate
#define SKIN_SMOOTHING_PULSE_LIMIT 2

// How far apart in time two pulses have to be. Given in units of milliseconds.
// Value here allows 50 Hz pulses from mains field, but disallows shorter pulses 
#define SKIN_SMOOTHING_PULSE_MIN_LENGTH 10

// Length of activation from one activation frame. Given in units of
// millisecond.
#define SKIN_ACTIVATION_LENGTH 500

// How long motor runs when it starts. Given in units of millisecond
#define MOTOR_RUNNING_TIME 20 * 1000

// Uncommenting this activates the serial debug mode
#define DEBUG

unsigned long lastSmoothingTime = 0;
unsigned long lastActivationTime = 0;
volatile unsigned long lastPulseReceivedTime = 0;

volatile uint16_t pulsesReceived = 0;

bool isMotorRunning = false;
unsigned long lastMotorChangeTime = 0;

#ifdef DEBUG
    int debugFrameNumber = 0;
#endif

void setup() {
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(SKIN_PIN, INPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_REVERSE_PIN, OUTPUT);

    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_REVERSE_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(SKIN_PIN), skinPulseReceived, FALLING); 

    #ifdef DEBUG
        Serial.begin(9600);
    #endif
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime > lastSmoothingTime + SKIN_SMOOTHING_FRAME_LENGTH) {
        if (pulsesReceived >= SKIN_SMOOTHING_PULSE_LIMIT) {
            activate(currentTime);
        }

        #ifdef DEBUG
            Serial.print("Smoothing frame ");
            Serial.print(debugFrameNumber++, DEC);
            Serial.print(" ends: ");
            Serial.print(pulsesReceived, DEC);
            Serial.print(" pulses received. Current time: ");
            Serial.print(currentTime, DEC);
            Serial.println();
        #endif
 
        lastSmoothingTime = currentTime;
        pulsesReceived = 0;
    }

    if (currentTime > lastActivationTime + SKIN_ACTIVATION_LENGTH) {
        digitalWrite(INDICATOR_PIN, LOW);
    }

    runMotor(currentTime);
}

void activate(uint32_t currentTime) {
    lastActivationTime = currentTime;
    digitalWrite(INDICATOR_PIN, HIGH);
}

void skinPulseReceived() {
    uint32_t currentTime = millis();
    if (currentTime < lastPulseReceivedTime + SKIN_SMOOTHING_PULSE_MIN_LENGTH) {
        return;
    }

    pulsesReceived++;
    lastPulseReceivedTime = currentTime;
}

void runMotor(unsigned long currentTime) {
    // TODO: This is just testing code, real run program has not been defined yet

    if (currentTime <= lastMotorChangeTime + MOTOR_RUNNING_TIME) {
        return;
    }

    isMotorRunning = !isMotorRunning;

    static bool isMotorDirectionForward = false;

    if (isMotorRunning) {
        isMotorDirectionForward = !isMotorDirectionForward;
    }
    int runPin = isMotorDirectionForward ? MOTOR_FORWARD_PIN : MOTOR_REVERSE_PIN;

    digitalWrite(MOTOR_ENABLE_PIN, isMotorRunning);
    digitalWrite(runPin, isMotorRunning);
    lastMotorChangeTime = currentTime;
}