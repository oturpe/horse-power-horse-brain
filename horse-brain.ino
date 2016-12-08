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
#define MOTOR_RUNNING_TIME (1 * 1000)

// Pin tied to trigger on the ultrasonic sensor.
#define SIGHT_TRIGGER_PIN  12
// Pin tied to echo on the ultrasonic sensor.
#define SIGHT_ECHO_FRONT_PIN A0
#define SIGHT_ECHO_BACK_PIN A1

// Interval of using sight. Given in units of millisecond
#define SIGHT_INTERVAL 500
// Maximum distance to be measured. Given in units of centimeter.
#define SIGHT_MAX_DISTANCE 400
// Maximum distance where the horse reacts to sightings. Given in units of
// centimeter.
#define SIGHT_ACTIVATION_DISTANCE 100

// Pin tied to output on Infrared sensor.
#define IR_FRONT_PIN A2
#define IR_BACK_PIN A3

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

//Indicator byte to listen to FRONT or BACK ECHO
byte isSightingFront = 0;

//Indicator byte to avoid "1" by triggering US
byte triggState = 0;

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

bool isMotorRunning = false;
unsigned long lastMotorChangeTime = 0;

#ifdef DEBUG
    int debugFrameNumber = 0;
    unsigned long sightLastPrintTime = 0;
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

    pinMode(SIGHT_ECHO_FRONT_PIN, INPUT);
    pinMode(SIGHT_ECHO_BACK_PIN, INPUT);
    pinMode(IR_FRONT_PIN, INPUT);
    pinMode(IR_BACK_PIN, INPUT);

    digitalWrite(SKIN_SEND_PIN, LOW);

    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_REVERSE_PIN, LOW);
    analogWrite(MOTOR_FREQUENCY_PIN, 0xff);

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

inline uint32_t ultrasonicDistance(int delay) {
    return delay / 2 / 29.1;
}

inline uint32_t ultrasonicDelay(int distance) {
    return distance * 2 * 29.1;
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

    // It triggers US sensor. It is "listening" to one sensor side at the time (FRONT or BACK)
    if (millis() - sightTriggeredTime > sightDelay) {
        if (isSightingFront == 0) {
            if (triggState == 0) {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                UltraSonic(SIGHT_ECHO_FRONT_PIN);
            }
            else {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                sightDistanceFront = UltraSonic(SIGHT_ECHO_FRONT_PIN);
            }
        }
        else {
            if (triggState == 0) {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                UltraSonic(SIGHT_ECHO_BACK_PIN);
            }
            else {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                sightDistanceBack = UltraSonic(SIGHT_ECHO_BACK_PIN);

                // TODO: Define real action on sighting
                bool isFrontSighting = sightDistanceFront <= sightThresholdDelay;
                bool isBackSighting = sightDistanceBack <= sightThresholdDelay;
                digitalWrite(INDICATOR_PIN, isFrontSighting || isBackSighting);

                #ifdef DEBUG
                    sightLastPrintTime = currentTime;

                    Serial.print("sight;front;");
                    Serial.print(ultrasonicDistance(sightDistanceFront), DEC);
                    if (isFrontSighting) {
                        Serial.print(";sighted");
                    }
                    Serial.println();
                    Serial.print("sight;back;");
                    Serial.print(ultrasonicDistance(sightDistanceBack), DEC);
                    if (isBackSighting) {
                        Serial.print(";sighted");
                    }
                    Serial.println();
                    }
                #endif
            }
        }
    }

    isPersonInFront = digitalRead(IR_FRONT_PIN);
    isPersonInBack = digitalRead(IR_BACK_PIN);

    runMotor(currentTime);
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

uint8_t motorFrequency = 0;
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
    motorFrequency += 8;

    digitalWrite(MOTOR_ENABLE_PIN, /*isMotorRunning*/LOW);
    digitalWrite(runPin, isMotorRunning);
    analogWrite(MOTOR_FREQUENCY_PIN, motorFrequency);
    lastMotorChangeTime = currentTime;
}

int UltraSonic(int PIN) { //Triggering US sensor
  int  DISTANCE = 0;
    
  if  (triggState == 0) {
    digitalWrite(SIGHT_TRIGGER_PIN, HIGH);
    sightTriggeredTime = millis();
    sightDelay = 10;
    triggState = 1;
  }

  else if (triggState == 1) {
    digitalWrite(SIGHT_TRIGGER_PIN, LOW);
    sightTriggeredTime = millis();
    sightDelay = SIGHT_INTERVAL;
    triggState = 0;

    // Returns the length of the pulse in microseconds (!!) or 0 if no complete
    // pulse was received within the timeout.
    DISTANCE = pulseIn(PIN, HIGH, sightMaxDelay);
    if (DISTANCE == 0) {
      DISTANCE  = sightMaxDelay;
    }
    if (isSightingFront == 0) {
      isSightingFront = 1;
    }
    else {
      isSightingFront = 0;
    }
    return (DISTANCE);
  }
}
