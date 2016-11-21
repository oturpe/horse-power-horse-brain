// Output pin for indicator led
#define INDICATOR_PIN 13

// Input pin for skin readings
#define SKIN_PIN 2

// Smoothing frame length. Given in units of millisecond
#define SMOOTHING_FRAME_LENGTH 300

// How many pulses have to be received in a frame to activate
#define SMOOTHING_PULSE_LIMIT 2

// How far apart in time two pulses have to be. Given in units of milliseconds.
// Value here allows 50 Hz pulses from mains field, but disallows shorter pulses 
#define SMOOTHING_PULSE_MIN_LENGTH 10

// Length of activation from one activation frame. Given in units of
// millisecond.
#define ACTIVATION_LENGTH 500

// Uncommenting this activates the serial debug mode
#define DEBUG

uint32_t lastSmoothingTime = 0;
uint32_t lastActivationTime = 0;
volatile uint32_t lastPulseReceivedTime = 0;

volatile uint16_t pulsesReceived = 0;

#ifdef DEBUG
    int debugFrameNumber = 0;
#endif

void setup() {	
    pinMode(INDICATOR_PIN, OUTPUT);
    pinMode(SKIN_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(SKIN_PIN), skinPulseReceived, FALLING); 

    #ifdef DEBUG
        Serial.begin(9600);
    #endif
}

void loop() {
    uint32_t currentTime = millis();
    if (currentTime > lastSmoothingTime + SMOOTHING_FRAME_LENGTH) {
        if (pulsesReceived >= SMOOTHING_PULSE_LIMIT) {
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

    if (currentTime > lastActivationTime + ACTIVATION_LENGTH) {
        digitalWrite(INDICATOR_PIN, LOW);
    }
}

void activate(uint32_t currentTime) {
	lastActivationTime = currentTime;
	digitalWrite(INDICATOR_PIN, HIGH);
}

void skinPulseReceived() {
    uint32_t currentTime = millis();
    if (currentTime < lastPulseReceivedTime + SMOOTHING_PULSE_MIN_LENGTH) {
        return;
    }
    
    pulsesReceived++;
    lastPulseReceivedTime = currentTime;
}