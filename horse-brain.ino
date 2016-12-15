#include "EnableInterrupt.h"

// Uncommenting this activates the serial debug mode
#define DEBUG

// Output pin for indicator led
#define INDICATOR_PIN 13

// Digital output pin for skin send
#define SKIN_SEND_PIN 2
// The amount of skin receive pins
#define SKIN_RECEIVE_PINS 4
// Digital input pins for skin receive
#define SKIN_RECEIVE_PIN_0 3
#define SKIN_RECEIVE_PIN_1 4
#define SKIN_RECEIVE_PIN_2 5
#define SKIN_RECEIVE_PIN_3 6

// Skin for reading values from bacterial fuel cell
#define FUEL_CELL_PIN A6
// The amount of samples to average when measuring fuel cell voltage
#define FUEL_CELL_SAMPLE_COUNT 10
// How often fuel cell values are printed in debug mode. Given in units of
// millisecond
#define FUEL_CELL_PRINT_INTERVAL 1000

// The minimum length of skin pulse that is accepted
#define SKIN_PULSE_MIN_LENGTH 10
// The maximum time to wait for a skin pulse. Given in units of microsecond
#define SKIN_PULSE_TIMEOUT (50 * 1000)
// The minimum time between two skin pulses. Given in units of millisecond
#define SKIN_PULSE_DELAY (4 * 1000)
//
#define SKIN_PULSE_THRESHOLD 1000

// Length of activation. Given in units of millisecond
// TODO: What is the activation actually?
#define ACTIVATION_LENGTH 1000

// Digital output pin for motor enable
#define MOTOR_ENABLE_PIN 10
// Digital output pin for motor forward run
#define MOTOR_FORWARD_PIN 9
// Digital output pin for motor reverse run
#define MOTOR_REVERSE_PIN 8
// Pwm output pin for motor frequency setting
#define MOTOR_FREQUENCY_PIN 11

// How long motor runs when it starts. Given in units of millisecond
#define MOTOR_RUNNING_TIME 500

// Pin tied to trigger on the ultrasonic sensor.
#define SIGHT_TRIGGER_PIN  12
// Pin tied to echo on the ultrasonic sensor.
#define SIGHT_ECHO_FRONT_PIN A1
#define SIGHT_ECHO_BACK_PIN A2

// Interval of using sight. Given in units of millisecond
#define SIGHT_INTERVAL 500
// Maximum distance to be measured. Given in units of centimeter.
#define SIGHT_MAX_DISTANCE 400
// Maximum distance where the horse reacts to sightings. Given in units of
// centimeter.
#define SIGHT_ACTIVATION_DISTANCE 100

// Pin tied to output on Infrared sensor.
#define IR_FRONT_PIN A3
#define IR_BACK_PIN A4

// Maximum distance we want to pin for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int sightMaxDelay = ultrasonicDelay(SIGHT_MAX_DISTANCE); 
// Maximum distance from sight that is reported as a sighting. Given in units of
// microsecond.
int sightThresholdDelay = ultrasonicDelay(SIGHT_ACTIVATION_DISTANCE);

//Measured distance US sensor
int sightDistanceFront = 0;
int sightDistanceBack = 0;

//If there is a person sensed by IR sensor.
byte isPersonInFront = 0;
byte isPersonInBack = 0;

//Timer for US sensor delay
unsigned long sightDelay = 0;

//When was US sensor triggerd the last time
unsigned long sightTriggeredTime = 0;

//Indicator boolean to listen to FRONT or BACK ECHO
bool isSightingFront = true;

//Indicator boolean to avoid "1" by triggering US
bool isSightSending = true;

enum SkinState {
    // Undefined state
    UNDEFINED,
    // Ready for sending new pulse
    READY,
    // Waiting for pulse reception
    WAITING,
    // Pulse has been received, but not processed
    RECEIVED,
    // Received pulse has been processed
    COOLING_DOWN,
    // Pulse fast received too quickly to get a reading
    ERROR_TOO_FAST,
    // Pulse was sent but never received
    ERROR_TIMEOUT
};

// The moment when last skin sense was started. Given in units of
// microsecond.
unsigned long skinPulseStartTime = 0;

struct Skin {
    // Receive pin
    uint8_t receivePin = 0;
    // Skin state
    SkinState state = READY;
    // Length of last received skin pulse. Given in units of microsecond.
    unsigned long pulseLength = 0;
    // The time when last skind pulse was received. Given in units of millisecond
    unsigned long pulseReceivedTime = 0;
};

Skin skins[SKIN_RECEIVE_PINS];

// The time when the horse was last activated. Given in units of millisecond
unsigned long lastActivationTime = 0;

// If motor is running at all
bool isMotorRunning = false;
// If motor movement direction is forward. Otherwise it is backward
bool isMotorDirectionForward = true;
// The last time motor state was changed. Given in units of millisecond
// measured from program start.
unsigned long lastMotorChangeTime = 0;
// Motor running frequency given as pwm duty cycle. Range [0xff ... 0xff] is
// linearly mapped to range defined by the inverter. It is expected to be
// [0.0 Hz .. 50.0 Hz].
uint8_t motorFrequency = 0;

#ifdef DEBUG
    unsigned long sightLastPrintTime = 0;
    unsigned long fuelCellLastPrintTime = 0;
#endif 

void setup() {
    pinMode(INDICATOR_PIN, OUTPUT);

    pinMode(SKIN_SEND_PIN, OUTPUT);

    pinMode(SKIN_RECEIVE_PIN_0, INPUT);
    pinMode(SKIN_RECEIVE_PIN_1, INPUT);
    pinMode(SKIN_RECEIVE_PIN_2, INPUT);
    pinMode(SKIN_RECEIVE_PIN_3, INPUT);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(MOTOR_FREQUENCY_PIN, OUTPUT);
    pinMode(SIGHT_TRIGGER_PIN, OUTPUT);

    pinMode(SIGHT_ECHO_FRONT_PIN, INPUT_PULLUP);
    pinMode(SIGHT_ECHO_BACK_PIN, INPUT_PULLUP);
    pinMode(IR_FRONT_PIN, INPUT);
    pinMode(IR_BACK_PIN, INPUT);
    pinMode(FUEL_CELL_PIN, A6);

    digitalWrite(SKIN_SEND_PIN, LOW);

    // Set motor to standstill in every possible way
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_REVERSE_PIN, HIGH);
    analogWrite(MOTOR_FREQUENCY_PIN, 0x00);

    enableInterrupt(SKIN_RECEIVE_PIN_0, skinPulseReceived_0, RISING);
    skins[0].receivePin = SKIN_RECEIVE_PIN_0;
    enableInterrupt(SKIN_RECEIVE_PIN_1, skinPulseReceived_1, RISING);
    skins[1].receivePin = SKIN_RECEIVE_PIN_1;
    enableInterrupt(SKIN_RECEIVE_PIN_2, skinPulseReceived_2, RISING);
    skins[2].receivePin = SKIN_RECEIVE_PIN_2;
    enableInterrupt(SKIN_RECEIVE_PIN_3, skinPulseReceived_3, RISING);
    skins[3].receivePin = SKIN_RECEIVE_PIN_3;

    #ifdef DEBUG
        Serial.begin(9600);
    #endif
}

void loop() {
    unsigned long currentTime = millis();

    for (int i = 0; i < SKIN_RECEIVE_PINS; i++) {
        unsigned long currentTimeMicros;
        Skin *skin = &skins[i];

        switch (skin->state) {
        case READY:
            break;
        case WAITING:
            currentTimeMicros = micros();
            if (currentTimeMicros > skinPulseStartTime + SKIN_PULSE_TIMEOUT) {
                // TODO: No idea why this does not work
                //skinState = ERROR_TIMEOUT;
            }
            break;
        case RECEIVED:
            if (skin->pulseLength > SKIN_PULSE_THRESHOLD) {
                // TOOD: Should a different thing happen for different skin elements?
                activate(currentTime);
            }

            #ifdef DEBUG
                serialPrintSkinReadingPrefix(i);
                Serial.println(skin->pulseLength, DEC);
            #endif

            skin->state = COOLING_DOWN;
            break;
        case COOLING_DOWN:
            break;
        case ERROR_TOO_FAST:
            #ifdef DEBUG
                serialPrintSkinReadingPrefix(i);
                Serial.println("Too fast");
            #endif
            skin->pulseReceivedTime = currentTime;
            skin->state = COOLING_DOWN;
            break;
        case ERROR_TIMEOUT:
            #ifdef DEBUG
                serialPrintSkinReadingPrefix(i);
                Serial.println("Timeout");
            #endif
            skin->pulseReceivedTime = currentTime;
            skin->state = COOLING_DOWN;
            break;
        }
    }

    SkinState skinCommonState = skins[0].state;
    for (int i = 0; i < SKIN_RECEIVE_PINS; i++) {
        if (skins[i].state != skinCommonState) {
            skinCommonState = UNDEFINED;
            break;
        }
    }

    switch (skinCommonState) {
    case READY:
        startSkinPulse();
        for (int i = 0; i < SKIN_RECEIVE_PINS; i++) {
            Skin * skin = &skins[i];
            skin->state = WAITING;
        }
        break;
    case COOLING_DOWN:
        digitalWrite(SKIN_SEND_PIN, LOW);
        bool areAllCooled = true;
        // Condition for sending another pulse.
        for (int i = 0; i < SKIN_RECEIVE_PINS; i++) {
            Skin * skin = &skins[i];
            if (currentTime <= skin->pulseReceivedTime + SKIN_PULSE_DELAY) {
                areAllCooled = false;
            }
        }
        if (areAllCooled) {
            for (int i = 0; i < SKIN_RECEIVE_PINS; i++) {
                Skin * skin = &skins[i];
                skin->state = READY;
            }
        }
        break;
    }

    if (currentTime > lastActivationTime + ACTIVATION_LENGTH) {
        digitalWrite(INDICATOR_PIN, LOW);
    }

    runSight(currentTime);

    int cellVoltage = runCellMeasurement(currentTime);

    isPersonInFront = digitalRead(IR_FRONT_PIN);
    isPersonInBack = digitalRead(IR_BACK_PIN);

    runMotor(currentTime);
}

/// \brief
///    Runs one step of sighting loop
///
/// \param currentTime
///    Current time in milliseconds
void runSight(unsigned long currentTime) {
    // It triggers US sensor. It is "listening" to one sensor side at the time (FRONT or BACK)
    if (currentTime <= sightTriggeredTime + sightDelay) {
        return;
    }

    if (isSightingFront) {
        if (isSightSending) {
            //When TRIGG you get 1 (!!), when you recive ECHO you get distance
            UltraSonic(SIGHT_ECHO_FRONT_PIN);
        }
        else {
            //When TRIGG you get 1 (!!), when you recive ECHO you get distance
            sightDistanceFront = UltraSonic(SIGHT_ECHO_FRONT_PIN);
            processSighting(sightDistanceFront, true);
        }
    }
    else {
        if (isSightSending) {
            //When TRIGG you get 1 (!!), when you recive ECHO you get distance
            UltraSonic(SIGHT_ECHO_BACK_PIN);
        }
        else {
            //When TRIGG you get 1 (!!), when you recive ECHO you get distance
            sightDistanceBack = UltraSonic(SIGHT_ECHO_BACK_PIN);
            processSighting(sightDistanceBack, false);
        }
    }
}

inline uint32_t ultrasonicDistance(int delay) {
    return delay / 2 / 29.1;
}

inline uint32_t ultrasonicDelay(int distance) {
    return distance * 2 * 29.1;
}

/// \brief
///    Triggers US sensor
///
/// \param echoPin
///    Pin where echo is listened
///
/// \return
///    Sighting delay when receiving. Constant value 0 when sending.
unsigned int UltraSonic(int echoPin) {
    unsigned int echoDelay = 0;

    if (isSightSending) {
        digitalWrite(SIGHT_TRIGGER_PIN, HIGH);
        sightTriggeredTime = millis();
        sightDelay = 10;
    }
    else {
        digitalWrite(SIGHT_TRIGGER_PIN, LOW);
        sightTriggeredTime = millis();
        sightDelay = SIGHT_INTERVAL;

        // Returns the length of the pulse in microseconds (!!) or 0 if no complete
        // pulse was received within the timeout.
        echoDelay = pulseIn(echoPin, HIGH, sightMaxDelay);
        if (echoDelay == 0) {
            echoDelay  = sightMaxDelay;
        }

        isSightingFront = !isSightingFront;
    }

    isSightSending = !isSightSending;

    return echoDelay;
}

/// \brief
///    Performs the actions required when something is sighted.
///
/// \param echoDelay
///    Echo deleay from distance sensor
///
/// \param isFront
///     If the sensor is the front sensor. Otherwise it is the back sensor
void processSighting(uint32_t echoDelay, bool isFront) {
    // TODO: Define real action on sighting
    bool isSighting = echoDelay <= sightThresholdDelay;
    digitalWrite(INDICATOR_PIN, isSighting);

    #ifdef DEBUG
        if (isFront) {
            Serial.print("sight;front;");
        }
        else {
            Serial.print("sight;back;");
        }

        Serial.print(ultrasonicDistance(echoDelay), DEC);
        if (isSighting) {
            Serial.print(";sighted");
        }
        Serial.println();
    #endif
}

void activate(unsigned long currentTime) {
    lastActivationTime = currentTime;
    digitalWrite(INDICATOR_PIN, HIGH);
}

void startSkinPulse() {
    skinPulseStartTime = micros();
    digitalWrite(SKIN_SEND_PIN, HIGH);
}

void skinPulseReceived_0() {
    skinPulseReceived(0);
}

void skinPulseReceived_1() {
    skinPulseReceived(1);
}

void skinPulseReceived_2() {
    skinPulseReceived(2);
}

void skinPulseReceived_3() {
    skinPulseReceived(3);
}

inline void skinPulseReceived(uint8_t skinIndex) {
    Skin * skin = &skins[skinIndex];
    // A false trigger, might happen when somebody is actually touching the skin
    if (skin->state != WAITING) {
         return;
    }

    skin->pulseReceivedTime = millis();
    unsigned long currentTime = micros();
    if (currentTime > skinPulseStartTime + SKIN_PULSE_MIN_LENGTH) {
        skin->pulseLength =  currentTime - skinPulseStartTime;
        skin->state = RECEIVED;
    }
    else {
        skin->state = ERROR_TOO_FAST;
    }
}

#ifdef DEBUG
    void serialPrintSkinReadingPrefix(uint8_t receiverIndex) {
        Serial.print("Skin reading;");
        Serial.print("[");
        Serial.print(receiverIndex, DEC);
        Serial.print("];");
    }
#endif

int runCellMeasurement(unsigned long currentTime) {
    unsigned int cellVoltage = 0;
    for (int i = 0; i < FUEL_CELL_SAMPLE_COUNT; i++) {
        cellVoltage += analogRead(FUEL_CELL_PIN);
    }
    cellVoltage /= FUEL_CELL_SAMPLE_COUNT;

    #ifdef DEBUG
        if (currentTime > fuelCellLastPrintTime + FUEL_CELL_PRINT_INTERVAL) {
            Serial.print("Cell voltage;");
            Serial.println(cellVoltage);
            fuelCellLastPrintTime = currentTime;
        }
    #endif

    return cellVoltage;
}

void runMotor(unsigned long currentTime) {
    // TODO: This is just testing code, real run program has not been defined yet

    if (currentTime <= lastMotorChangeTime + MOTOR_RUNNING_TIME) {
        return;
    }

    isMotorRunning = true;

    // TEMP: Just testing code
    #define MOTOR_FREQUENCY_INCREMENT 8
    uint8_t newMotorFrequency = motorFrequency + MOTOR_FREQUENCY_INCREMENT;
    if (newMotorFrequency < motorFrequency) {
        isMotorDirectionForward = !isMotorDirectionForward;
    }
    motorFrequency = newMotorFrequency;

    // Note: interface to inverter's 24 V logic inverts values, thus LOW on output
    // is seen as HIGH on inverter inputs and vice versa.
    digitalWrite(MOTOR_ENABLE_PIN, !isMotorRunning);
    digitalWrite(MOTOR_FORWARD_PIN, !isMotorRunning || !isMotorDirectionForward);
    digitalWrite(MOTOR_REVERSE_PIN, !isMotorRunning || isMotorDirectionForward);
    analogWrite(MOTOR_FREQUENCY_PIN, motorFrequency);

    lastMotorChangeTime = currentTime;
}
