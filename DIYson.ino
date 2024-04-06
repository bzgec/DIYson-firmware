/*
 * Setup Arduino IDE for Adafruit QT Py SAMD21 board
 *   1. Add additional board json file - https://learn.adafruit.com/adafruit-qt-py/arduino-ide-setup
 *   2. Install SAMD support - https://learn.adafruit.com/adafruit-qt-py/using-with-arduino-ide
 * Add Adafruit_FreeTouch library
 *   1. Download latest Adafruit_FreeTouch library .zip -
 * https://github.com/adafruit/Adafruit_FreeTouch/releases
 *   2. Import library to sketch - "Sketch > Import"
 */
#include "Adafruit_FreeTouch.h"

#define LED_BUILDIN 13
#define PIN_DAC A0

#define TOUCH_PRESSED_THRESHOLD 800u
#define TOUCH_PRESSED_HYSTERESIS 100u
#define TOUCH_PRESSED_THRESHOLD_HIGH (TOUCH_PRESSED_THRESHOLD + (TOUCH_PRESSED_HYSTERESIS / 2u))
#define TOUCH_PRESSED_THRESHOLD_LOW (TOUCH_PRESSED_THRESHOLD - (TOUCH_PRESSED_HYSTERESIS / 2u))

#define PIN_TOGGLE A8
#define PIN_INC A9
#define PIN_DEC A10

#define DAC_VAL_MAX 255u

typedef enum {
    BRIGHTNESS_OFF = 0u,
    BRIGHTNESS_1 = 1u,
    BRIGHTNESS_2 = 2u,
    BRIGHTNESS_3 = 3u,
    BRIGHTNESS_MAX = 4u,
    BRIGHTNESS_COUNT = 5u,
} BRIGHTNESS_E;

// Comment out to stop debug message printing
// #define DEBUG_TOUCH_PINS

Adafruit_FreeTouch touch_toggle
        = Adafruit_FreeTouch(PIN_TOGGLE, OVERSAMPLE_32, RESISTOR_50K, FREQ_MODE_SPREAD);
Adafruit_FreeTouch touch_inc
        = Adafruit_FreeTouch(PIN_INC, OVERSAMPLE_32, RESISTOR_50K, FREQ_MODE_SPREAD);
Adafruit_FreeTouch touch_dec
        = Adafruit_FreeTouch(PIN_DEC, OVERSAMPLE_32, RESISTOR_50K, FREQ_MODE_SPREAD);

bool ledOn = false;
uint8_t brightness = BRIGHTNESS_1;

// The setup routine runs once when you press reset:
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("DIYson LED controller");

    pinMode(LED_BUILDIN, OUTPUT);

    // Turn off the LED
    analogWrite(PIN_DAC, 0);

    // Initialize Touch sensors
    if(!touch_toggle.begin()) {
        Serial.println("Failed to init touch_toggle");
    }
    if(!touch_inc.begin()) {
        Serial.println("Failed to init touch_inc");
    }
    if(!touch_dec.begin()) {
        Serial.println("Failed to init touch_dec");
    }
}

// The loop routine runs over and over again forever:
bool isPressedState_previous_toggle;
bool isPressedState_previous_inc;
bool isPressedState_previous_dec;
void loop() {
    uint16_t touch_value_toggle = 0;
    uint16_t touch_value_inc = 0;
    uint16_t touch_value_dec = 0;

    bool isPressed_toggle = 0;
    bool isPressed_inc = 0;
    bool isPressed_dec = 0;
    bool isPressed_togglePrev = 0;
    bool isPressed_incPrev = 0;
    bool isPressed_decPrev = 0;

    bool risingEdgeState;

    wait_for_power_on();
    digitalWrite(LED_BUILDIN, 0);
    Serial.print("Brightness (ON): ");
    Serial.println(brightness);
    setLed((BRIGHTNESS_E)brightness);

    while(ledOn == true) {
        risingEdgeState = isRisingEdge(&touch_toggle, &isPressedState_previous_toggle);
        if(risingEdgeState == true) {
            ledOn = false;
            digitalWrite(LED_BUILDIN, 1);
            Serial.print("Brightness (OFF): ");
            Serial.println(brightness);
            setLed(BRIGHTNESS_OFF);
            break;
        }

        risingEdgeState = isRisingEdge(&touch_inc, &isPressedState_previous_inc);
        if(risingEdgeState == true) {
            if(brightness < BRIGHTNESS_MAX) {
                brightness++;
                Serial.print("Brightness (++): ");
                Serial.println(brightness);
                setLed((BRIGHTNESS_E)brightness);
            }
        }

        risingEdgeState = isRisingEdge(&touch_dec, &isPressedState_previous_dec);
        if(risingEdgeState == true) {
            if(brightness > BRIGHTNESS_OFF) {
                brightness--;
                Serial.print("Brightness (--): ");
                Serial.println(brightness);
                setLed((BRIGHTNESS_E)brightness);
            }
        }
    }

#ifdef DEBUG_TOUCH_PINS
    touch_value_toggle = touch_toggle.measure();
    touch_value_inc = touch_inc.measure();
    touch_value_dec = touch_dec.measure();

    Serial.print(touch_value_toggle);
    Serial.print(",");

    Serial.print(touch_value_inc);
    Serial.print(",");

    Serial.print(touch_value_dec);
    Serial.print("\n");
#endif
}

inline bool isTouchPinPressedWithHysteresis(uint16_t measurement, bool previousState) {
    bool pinPressed = previousState;

    if(previousState == false) {
        if(measurement > TOUCH_PRESSED_THRESHOLD_HIGH) {
            Serial.print("Meas: ");
            Serial.print(measurement);
            Serial.print(", previousState: ");
            Serial.println(previousState);
            pinPressed = true;
        }
    } else {
        // previousState == true
        if(measurement < TOUCH_PRESSED_THRESHOLD_LOW) {
            Serial.print("Meas: ");
            Serial.print(measurement);
            Serial.print(", previousState: ");
            Serial.println(previousState);
            pinPressed = false;
        }
    }

    return pinPressed;
}

void setLed(BRIGHTNESS_E brightness) {
    uint8_t dacBrightnessValues[BRIGHTNESS_COUNT] = {
            0u,           // BRIGHTNESS_OFF
            63u,          // BRIGHTNESS_1
            126u,         // BRIGHTNESS_2
            189u,         // BRIGHTNESS_3
            DAC_VAL_MAX,  // BRIGHTNESS_MAX
    };

    analogWrite(PIN_DAC, dacBrightnessValues[brightness]);
}

bool isRisingEdge(Adafruit_FreeTouch *pTouchPin, bool *pPrevTouchSt) {
    uint16_t touch_value = 0;
    bool isPressed = false;
    bool riseEdge = false;

    touch_value = pTouchPin->measure();
    isPressed = isTouchPinPressedWithHysteresis(touch_value, *pPrevTouchSt);
    if(isPressed == true && *pPrevTouchSt == false) {
        riseEdge = true;
    }

    *pPrevTouchSt = isPressed;

    return riseEdge;
}
void wait_for_power_on() {
    bool risingEdgeState;
    // bool isPressedState_previous;

    do {
        risingEdgeState = isRisingEdge(&touch_toggle, &isPressedState_previous_toggle);
    } while(risingEdgeState == false);

    ledOn = true;
}
