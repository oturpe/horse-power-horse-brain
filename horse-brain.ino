// Output pin for indicator led
#define INDICATOR_PIN 13

// Digital output pin for skin send
#define SKIN_SEND_PIN 3
// Digital input pin for skin receive
#define SKIN_RECEIVE_PIN 2

// The minimum length of skin pulse that is accepted
#define SKIN_PULSE_MIN_LENGTH 10
// The maximum time to wait for a skin pulse. Given in units of microsecond
#define SKIN_PULSE_TIMEOUT (50 * 1000)
// The minimum time between two skin pulses. Given in units of millisecond
#define SKIN_PULSE_DELAY 400
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
int MAX_DISTANCE = 400; 

//Measured distance US sensor
int DISTANCE_FRONT = 0;
int DISTANCE_BACK = 0;

//If there is a person sensed by IR sensor.
byte PERSON_FRONT = 0;
byte PERSON_BACK = 0;

//Timer for US sensor delay
unsigned long T_DELAY_US = 0;

//When was US sensor triggerd the last time
unsigned long T_TRIGG = 0;

//Indicator byte to listen to FRONT or BACK ECHO
byte ECHO_PIN = 0;

//Indicator byte to avoid "1" by triggering US
byte TRIGG_STATE = 0;

// Uncommenting this activates the serial debug mode
#define DEBUG

enum SkinState {
    // Ready for sending new pulse
    READY,
    // Waiting for pulse reception
    WAITING,
    // Pulse has been received, but not processed
    RECEIVED,
    // Received pulse has been processed
    PROCESSED,
    // Pulse fast received too quickly to get a reading
    ERROR_TOO_FAST,
    // Pulse was sent but never received
    ERROR_TIMEOUT
};

// Skin state
SkinState skinState = READY;
// The moment when last skin sense was started. Given in units of
// microsecond.
unsigned long skinPulseStartTime = 0;
// Length of last received skin pulse. Given in units of microsecond.
unsigned long skinPulseLength = 0;
// The time when last skind pulse was received. Given in units of millisecond
unsigned long skinPulseReceivedTime = 0;

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
    pinMode(SKIN_RECEIVE_PIN, INPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_REVERSE_PIN, OUTPUT);
    pinMode(MOTOR_FREQUENCY_PIN, OUTPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    pinMode(ECHO_FRONT_PIN, INPUT);
    pinMode(ECHO_BACK_PIN, INPUT);
    pinMode(IR_FRONT_PIN, INPUT);
    pinMode(IR_BACK_PIN, INPUT);

    digitalWrite(SKIN_SEND_PIN, LOW);

    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_REVERSE_PIN, LOW);
    analogWrite(MOTOR_FREQUENCY_PIN, 0xff);

    MAX_DISTANCE = MAX_DISTANCE * 2 * 29.1; // Turn MAX_DISTANCE from CM into "TIME".
    
    attachInterrupt(digitalPinToInterrupt(SKIN_RECEIVE_PIN), skinPulseReceived, RISING); 

    #ifdef DEBUG
        Serial.begin(9600);
    #endif
}

void loop() {
    unsigned long currentTime = millis();

    unsigned long currentTimeMicros;
    switch (skinState) {
    case READY:
        startSkinPulse();
        skinState = WAITING;
        break;
    case WAITING:
        currentTimeMicros = micros();
        if (currentTimeMicros > skinPulseStartTime + SKIN_PULSE_TIMEOUT) {
            //skinState = ERROR_TIMEOUT;
        }
        break;
    case RECEIVED:
        digitalWrite(SKIN_SEND_PIN, LOW);

        if (skinPulseLength > SKIN_PULSE_THRESHOLD) {
            activate(currentTime);
        }

        #ifdef DEBUG
            Serial.print("Skin reading:");
            Serial.println(skinPulseLength);
        #endif

        skinState = PROCESSED;
        break;
    case PROCESSED:
        // Condition for sending another pulse.
        if (currentTime > skinPulseReceivedTime + SKIN_PULSE_DELAY) {
            skinState = READY;
        }
        break;
    case ERROR_TOO_FAST:
        #ifdef DEBUG
            Serial.println("Skin reading: Too fast");
        #endif
        skinPulseReceivedTime = currentTime;
        digitalWrite(SKIN_SEND_PIN, LOW);
        skinState = PROCESSED;
        break;
    case ERROR_TIMEOUT:
        #ifdef DEBUG
            Serial.println("Skin reading: Timeout");
        #endif
        skinPulseReceivedTime = currentTime;
        digitalWrite(SKIN_SEND_PIN, LOW);
        skinState = PROCESSED;
        break;
    }

    if (currentTime > lastActivationTime + ACTIVATION_LENGTH) {
        digitalWrite(INDICATOR_PIN, LOW);
    }

    runMotor(currentTime);
    
  if (millis() - T_TRIGG > T_DELAY_US) { // It triggers US sensor. It is "listeneing" to one sensor side at the time (FRONT or BACK)
    if (ECHO_PIN == 0) {
      if (TRIGG_STATE == 0) {
        UltraSonic(ECHO_FRONT_PIN); //When TRIGG you get 1 (!!), when you recive ECHO you get distance
      }
      else {
        DISTANCE_FRONT = UltraSonic(ECHO_FRONT_PIN); //When TRIGG you get 1 (!!), when you recive ECHO you get distance
      }
    }
    else {
      if (TRIGG_STATE == 0) {
        UltraSonic(ECHO_BACK_PIN); //When TRIGG you get 1 (!!), when you recive ECHO you get distance
      }
      else {
        DISTANCE_BACK = UltraSonic(ECHO_BACK_PIN); //When TRIGG you get 1 (!!), when you recive ECHO you get distance
      }
    }

    PERSON_FRONT = digitalRead(IR_FRONT_PIN);
    PERSON_BACK = digitalRead(IR_BACK_PIN);
}

void activate(unsigned long currentTime) {
    lastActivationTime = currentTime;
    digitalWrite(INDICATOR_PIN, HIGH);
}

void startSkinPulse() {
    skinPulseStartTime = micros();
    digitalWrite(SKIN_SEND_PIN, HIGH);
}

void skinPulseReceived() {
    // A false trigger, might happen when somebody is actually touching the skin
    if (skinState != WAITING) {
         return;
    }

    skinPulseReceivedTime = millis();
    unsigned long currentTime = micros();
    if (currentTime > skinPulseStartTime + SKIN_PULSE_MIN_LENGTH) {
        skinPulseLength =  currentTime - skinPulseStartTime;
        skinState = RECEIVED;
    }
    else {
        skinState = ERROR_TOO_FAST;
    }
}

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
    
  if  (TRIGG_STATE == 0) {
    digitalWrite(TRIGGER_PIN, HIGH);
    T_TRIGG = millis();
    T_DELAY_US = 10;
    TRIGG_STATE = 1;
  }

  else if (TRIGG_STATE == 1) {
    digitalWrite(TRIGGER_PIN, LOW);
    T_TRIGG = millis();
    T_DELAY_US = 0;
    TRIGG_STATE = 0;

    DISTANCE = pulseIn(PIN, HIGH, MAX_DISTANCE); //Returns the length of the pulse in microseconds (!!) or 0 if no complete pulse was received within the timeout.
    if (DISTANCE == 0) {
      DISTANCE  = MAX_DISTANCE;
    }
    if (ECHO_PIN == 0) {
      ECHO_PIN = 1;
    }
    else {
      ECHO_PIN = 0;
    }
    return (DISTANCE);
  }
}
