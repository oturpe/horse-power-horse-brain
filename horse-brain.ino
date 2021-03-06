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

// The thresholds of skin readings for activation
#define SKIN_HEAD_THRESHOLD 500
#define SKIN_LEFT_THRESHOLD 500
#define SKIN_RIGHT_THRESHOLD 500
#define SKIN_BODY_THRESHOLD 500

// Pin tied to output on Infrared sensor.
#define IR_FRONT_PIN A4
#define IR_BACK_PIN A3

// Skin for reading values from bacterial fuel cell
#define FUEL_CELL_PIN A6
// The amount of samples to average when measuring fuel cell voltage
#define FUEL_CELL_SAMPLE_COUNT 10
// How often fuel cell values are printed in debug mode. Given in units of
// millisecond
#define FUEL_CELL_PRINT_INTERVAL 1000L

// Thresholds that fuel cell measurements have to exceed to trigger each
// emotional state.
#define EMOTION_THRESHOLD_CALM 100
#define EMOTION_THRESHOLD_ACTIVE 120
#define EMOTION_THRESHOLD_WILD 150

// Speed of movement associated with each emotion. Given in linear units in
// range [0x00, 0xff].
#define EMOTION_SPEED_SLEEPY 80
#define EMOTION_SPEED_CALM 100
#define EMOTION_SPEED_ACTIVE 120
#define EMOTION_SPEED_WILD 150

// Amount of time before horse wants to move associated with each emotion.
// Given in units of millisecond.
#define EMOTION_REST_PERIOD_SLEEPY (1 * 60 * 1000L)
#define EMOTION_REST_PERIOD_CALM (20 * 1000L)
#define EMOTION_REST_PERIOD_ACTIVE (10 * 1000L)
#define EMOTION_REST_PERIOD_WILD (5 * 1000L)

// How long restlessness continues after it starts. Given in units of
// millisecond
#define EMOTION_RESTLESS_PERIOD (10 * 1000L)

// The minimum length of skin pulse that is accepted
#define SKIN_PULSE_MIN_LENGTH 10
// The maximum time to wait for a skin pulse. Given in units of microsecond
#define SKIN_PULSE_TIMEOUT (50 * 1000L)
// The minimum time between two skin pulses. Given in units of millisecond
#define SKIN_PULSE_DELAY (4 * 1000L)
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
#define SIGHT_INTERVAL 1500L
// Maximum distance to be measured. Given in units of centimeter.
#define SIGHT_MAX_DISTANCE 400
// Maximum distance where the horse reacts to sightings. Given in units of
// centimeter.
#define SIGHT_ACTIVATION_DISTANCE 50

// How long the motion sense remains active since detecting motion. Given in
// units of millisecond
#define IR_ACTIVATION_PERIOD (5 * 1000L)

// Interval between printing sense values in debug mode. Given in units of millisecond
#define SENSE_PRINT_INTERVAL ( 2* 1000L)

// Maximum distance we want to pin for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int sightMaxDelay = ultrasonicDelay(SIGHT_MAX_DISTANCE); 
// Maximum distance from sight that is reported as a sighting. Given in units of
// microsecond.
int sightThresholdDelay = ultrasonicDelay(SIGHT_ACTIVATION_DISTANCE);

//Measured distance US sensor
int sightDistanceFront = 0;
int sightDistanceBack = 0;

//Timer for US sensor delay
unsigned long sightDelay = 0;

//When was US sensor triggerd the last time
unsigned long sightTriggeredTime = 0;

//Indicator boolean to listen to FRONT or BACK ECHO
bool isSightingFront = true;

//Indicator boolean to avoid "1" by triggering US
bool isSightSending = true;

// Instants when infra red motion sensor last detected movement. Given in units
// of millisecond measured from program start
unsigned long lastIrFrontDetection = 0;
unsigned long lastIrBackDetection = 0;

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

struct Senses {
    bool skinHeadActivated;
    bool skinLeftActivated;
    bool skinRightActivated;
    bool skinBodyActivated;

    bool sightFrontActivated;
    bool sightBackActivated;

    bool personFrontActivated;
    bool personBackActivated;
};

Senses senses;

enum Emotion {
    SLEEPY = 0,
    CALM = 1,
    ACTIVE = 2,
    WILD = 3
};

Emotion emotion = SLEEPY;

uint8_t emotionSpeed() {
    switch (emotion) {
    case SLEEPY:
        return EMOTION_SPEED_SLEEPY;
    case CALM:
        return EMOTION_SPEED_CALM;
    case ACTIVE:
        return EMOTION_SPEED_ACTIVE;
    case WILD:
        return EMOTION_SPEED_WILD;
    }

    return EMOTION_SPEED_CALM;
}


unsigned long emotionRestPeriod() {
    switch (emotion) {
    case SLEEPY:
        return EMOTION_REST_PERIOD_SLEEPY;
    case CALM:
        return EMOTION_REST_PERIOD_CALM;
    case ACTIVE:
        return EMOTION_REST_PERIOD_ACTIVE;
    case WILD:
        return EMOTION_REST_PERIOD_WILD;
    }
}


// The time when the horse was last activated. Given in units of millisecond
// measured from program start.
unsigned long lastActivationTime = 0;
// The moment of time when the horse calms down. Given in units of millisecond
// measured from program start
unsigned long calmingTime = 0;
// If restlessness is directed towards going forward. Otherwise it is directed
// towards going backward.
bool isRestlessForward;

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
    unsigned long senseLastPrintTime = 0;
#endif 

void setup() {
    pinMode(INDICATOR_PIN, OUTPUT);

    /*
    pinMode(SKIN_SEND_PIN, OUTPUT);

    pinMode(SKIN_RECEIVE_PIN_0, INPUT);
    pinMode(SKIN_RECEIVE_PIN_1, INPUT);
    pinMode(SKIN_RECEIVE_PIN_2, INPUT);
    pinMode(SKIN_RECEIVE_PIN_3, INPUT);
    */

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

    /*
    enableInterrupt(SKIN_RECEIVE_PIN_0, skinPulseReceived_0, RISING);
    skins[0].receivePin = SKIN_RECEIVE_PIN_0;
    enableInterrupt(SKIN_RECEIVE_PIN_1, skinPulseReceived_1, RISING);
    skins[1].receivePin = SKIN_RECEIVE_PIN_1;
    enableInterrupt(SKIN_RECEIVE_PIN_2, skinPulseReceived_2, RISING);
    skins[2].receivePin = SKIN_RECEIVE_PIN_2;
    enableInterrupt(SKIN_RECEIVE_PIN_3, skinPulseReceived_3, RISING);
    skins[3].receivePin = SKIN_RECEIVE_PIN_3;
    */

    enableInterrupt(IR_FRONT_PIN, irDetectedFront, RISING);
    enableInterrupt(IR_BACK_PIN, irDetectedBack, RISING);

    // Seed the pseudo random number generator with static voltage noise in
    // unconnected A0 pin.
    pinMode(A0, INPUT);
    randomSeed(analogRead(A0));
    pinMode(A0, OUTPUT);

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
            switch (i) {
            case 0:
                senses.skinHeadActivated = skin->pulseLength > SKIN_HEAD_THRESHOLD;
                break;
            case 1:
                senses.skinLeftActivated = skin->pulseLength > SKIN_LEFT_THRESHOLD;
                break;
            case 2:
                senses.skinRightActivated = skin->pulseLength > SKIN_RIGHT_THRESHOLD;
                break;
            case 3:
                senses.skinBodyActivated = skin->pulseLength > SKIN_BODY_THRESHOLD;
                break;
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
        //startSkinPulse();
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

    runSight(currentTime);
    runCellMeasurement(currentTime);
    runMotionDetection(currentTime);
    runMotor(currentTime);

    #ifdef DEBUG
        if (currentTime > senseLastPrintTime + SENSE_PRINT_INTERVAL) {
            senseLastPrintTime = currentTime;

            Serial.print("Senses;");
            if (senses.skinHeadActivated) {
                Serial.print("skinHead-");
            }
            if (senses.skinLeftActivated) {
                Serial.print("skinLeft-");
            }
            if (senses.skinRightActivated) {
                Serial.print("skinRight-");
            }
            if (senses.skinBodyActivated) {
                Serial.print("skinBody-");
            }
            if (senses.sightFrontActivated) {
                Serial.print("frontSight-");
            }
            if (senses.sightBackActivated) {
                Serial.print("backSight-");
            }
            if (senses.personFrontActivated) {
                Serial.print("personFront-");
            }
            if (senses.personBackActivated) {
                Serial.print("personBack-");
            }
            Serial.println();
        }
    #endif
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

    if (isFront) {
        senses.sightFrontActivated = isSighting;
    }
    else {
        senses.sightBackActivated = isSighting;
    }

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

void runCellMeasurement(unsigned long currentTime) {
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

    if (cellVoltage > EMOTION_THRESHOLD_WILD) {
        emotion = WILD;
    }
    else if (cellVoltage > EMOTION_THRESHOLD_ACTIVE) {
        emotion = ACTIVE;
    }
    else if (cellVoltage > EMOTION_THRESHOLD_CALM) {
        emotion = CALM;
    }
    else {
        emotion = SLEEPY;
    }

    emotion = ACTIVE;
}

void irDetectedFront() {
    lastIrFrontDetection = millis();
}

void irDetectedBack() {
    lastIrBackDetection = millis();
}

void runMotionDetection(unsigned long currentTime) {

    senses.personFrontActivated = (
        (lastIrFrontDetection > 0) 
        && (currentTime <= lastIrFrontDetection + IR_ACTIVATION_PERIOD)
    );
    senses.personBackActivated = (
        (lastIrBackDetection > 0)
        && (currentTime <= lastIrBackDetection + IR_ACTIVATION_PERIOD)
    );
}

void signalInverter(
    bool isMoving,
    bool isGoingForward,
    bool isGoingBackward,
    uint8_t speed
) {
    // Note: interface to inverter's 24 V logic inverts values, thus LOW on output
    // is seen as HIGH on inverter inputs and vice versa.
    digitalWrite(MOTOR_ENABLE_PIN, !isMoving);
    digitalWrite(MOTOR_FORWARD_PIN, !isGoingForward);
    digitalWrite(MOTOR_REVERSE_PIN, !isGoingBackward);
    analogWrite(MOTOR_FREQUENCY_PIN, speed);
}

void runMotor(unsigned long currentTime) {
    bool isFeelingGoingForward = 
        senses.skinHeadActivated
        || senses.skinBodyActivated
        || senses.personFrontActivated
    ;

    bool isFeelingGoingBackward = 
        senses.skinLeftActivated
        || senses.skinRightActivated
        || senses.personBackActivated
    ;

    uint8_t speed = emotionSpeed();
    unsigned long restPeriod = emotionRestPeriod();

    bool isBecomingRestless = currentTime > lastActivationTime + restPeriod;
    if (isBecomingRestless) {
        calmingTime = currentTime + EMOTION_RESTLESS_PERIOD;
        isRestlessForward = random(2) == 1;
    }

    bool isFeelingMoving = 
        isFeelingGoingForward
        || isFeelingGoingBackward
        || isBecomingRestless
        || (currentTime <= calmingTime)
    ;

    if (isFeelingMoving) {
        lastActivationTime = currentTime;
    }

    // If he wants to move but has no preference for direction, he will choose
    // a random direction.
    if (isFeelingMoving && !isFeelingGoingForward && !isFeelingGoingBackward) {
        isFeelingGoingForward = isRestlessForward;
        isFeelingGoingBackward = !isRestlessForward;
    }

    // Safety: Do no go into direction where an obstacle is spotted
    bool isForwardMovementSafe = false;
    bool isBackwardMovementSafe = false;
    if (!senses.sightFrontActivated) {
        isForwardMovementSafe = true;
    }
    if (!senses.sightBackActivated) {
        isBackwardMovementSafe = true;
    }

    signalInverter(
        isFeelingMoving,
        isFeelingGoingForward && isForwardMovementSafe,
        isFeelingGoingBackward && isBackwardMovementSafe,
        speed
    );
}

