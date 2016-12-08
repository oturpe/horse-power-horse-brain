#include "EnableInterrupt.h"

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
#define TRIGGER_PIN  7
// Pin tied to echo on the ultrasonic sensor.
#define ECHO_FRONT_PIN     6  
#define ECHO_BACK_PIN     5
// Pin tied to output on Infrared sensor.
#define IR_FRONT_PIN 4
#define IR_BACK_PIN 3

// Maximum distance we want to pin for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int sightMaxDistance = 400; 

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

// Uncommenting this activates the serial debug mode
#define DEBUG

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
    pinMode(TRIGGER_PIN, OUTPUT);

    pinMode(ECHO_FRONT_PIN, INPUT);
    pinMode(ECHO_BACK_PIN, INPUT);
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

    sightMaxDistance = sightMaxDistance * 2 * 29.1; // Turn sightMaxDistance from CM into "TIME".

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

    runMotor(currentTime);

    // It triggers US sensor. It is "listening" to one sensor side at the time (FRONT or BACK)
    if (millis() - sightTriggeredTime > sightDelay) {
        if (isSightingFront == 0) {
            if (triggState == 0) {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                UltraSonic(ECHO_FRONT_PIN);
            }
            else {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                sightDistanceFront = UltraSonic(ECHO_FRONT_PIN);
            }
        }
        else {
            if (triggState == 0) {
                //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                UltraSonic(ECHO_BACK_PIN);
            }
            else {
                 //When TRIGG you get 1 (!!), when you recive ECHO you get distance
                 sightDistanceBack = UltraSonic(ECHO_BACK_PIN);
            }
        }
    }

    isPersonInFront = digitalRead(IR_FRONT_PIN);
    isPersonInBack = digitalRead(IR_BACK_PIN);
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

void skinPulseReceived(uint8_t skinIndex) {
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
    digitalWrite(TRIGGER_PIN, HIGH);
    sightTriggeredTime = millis();
    sightDelay = 10;
    triggState = 1;
  }

  else if (triggState == 1) {
    digitalWrite(TRIGGER_PIN, LOW);
    sightTriggeredTime = millis();
    sightDelay = 0;
    triggState = 0;

    // Returns the length of the pulse in microseconds (!!) or 0 if no complete
    // pulse was received within the timeout.
    DISTANCE = pulseIn(PIN, HIGH, sightMaxDistance);
    if (DISTANCE == 0) {
      DISTANCE  = sightMaxDistance;
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
