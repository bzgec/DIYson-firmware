/*
 * Setup Arduino IDE for Adafruit QT Py SAMD21 board
 *   1. Add additional board json file - https://learn.adafruit.com/adafruit-qt-py/arduino-ide-setup
 *   2. Install SAMD support - https://learn.adafruit.com/adafruit-qt-py/using-with-arduino-ide
 * Add Adafruit_FreeTouch library
 *   1. Download latest Adafruit_FreeTouch library .zip - https://github.com/adafruit/Adafruit_FreeTouch/releases
 *   2. Import library to sketch - "Sketch > Import"
 */
#include "Adafruit_FreeTouch.h"

#define SEEEDUINO_XIAO
// #define ADAFRUIT_QT_PY_SAMD21

#if((defined(SEEEDUINO_XIAO) + defined(ADAFRUIT_QT_PY_SAMD21)) >= 2)
#error "Only one board can be defined"
#endif

#if((defined(SEEEDUINO_XIAO) + defined(ADAFRUIT_QT_PY_SAMD21)) == 0)
#error "At least one board must be defined"
#endif

////////////////////////////////////////////////////////////////////////////////
// Pins
////////////////////////////////////////////////////////////////////////////////
#ifdef SEEEDUINO_XIAO
#define PIN_INDICATOR_LED 13
#define PIN_DAC A0

#define PIN_TOGGLE A8
#define PIN_INC A9
#define PIN_DEC A10
#endif  // SEEEDUINO_XIAO

#ifdef ADAFRUIT_QT_PY_SAMD21
#define PIN_INDICATOR_LED 13
#define PIN_DAC A0

#define PIN_TOGGLE A8
#define PIN_INC A9
#define PIN_DEC A10
#endif  // ADAFRUIT_QT_PY_SAMD21
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Touch filtering configuration
////////////////////////////////////////////////////////////////////////////////
#define TOUCH_PRESSED_THRESHOLD 700u
#define TOUCH_PRESSED_HYSTERESIS 50u
#define TOUCH_PRESSED_THRESHOLD_HIGH (TOUCH_PRESSED_THRESHOLD + (TOUCH_PRESSED_HYSTERESIS / 2u))
#define TOUCH_PRESSED_THRESHOLD_LOW (TOUCH_PRESSED_THRESHOLD - (TOUCH_PRESSED_HYSTERESIS / 2u))

#define FILTER_EMA_ALFA 0.1f  // When this is 1, there is no filtering
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Debug
////////////////////////////////////////////////////////////////////////////////
// Used indicator LED to show when DAC is not 0
//#define DEBUG_INDICATOR_LED

// Comment out to stop debug message printing
// Enable only one debug message at a time
// #define DEBUG_TOUCH_VALUES
// #define DEBUG_PIN_STATES

#if((defined(DEBUG_TOUCH_VALUES) + defined(DEBUG_PIN_STATES)) >= 2)
#error "Only one debug value spamming can be active at the same time"
#endif
////////////////////////////////////////////////////////////////////////////////

#define DAC_VAL_MAX 255u

// LED is active low
#define DEBUG_LED_ST_ON 0
#define DEBUG_LED_ST_OFF 1

typedef enum {
    BRIGHTNESS_OFF = 0u,
    BRIGHTNESS_1 = 1u,
    BRIGHTNESS_2 = 2u,
    BRIGHTNESS_3 = 3u,
    BRIGHTNESS_MAX = 4u,
    BRIGHTNESS_COUNT = 5u,
} BRIGHTNESS_E;

Adafruit_FreeTouch touch_power
        = Adafruit_FreeTouch(PIN_TOGGLE, OVERSAMPLE_2, RESISTOR_50K, FREQ_MODE_SPREAD);
Adafruit_FreeTouch touch_inc
        = Adafruit_FreeTouch(PIN_INC, OVERSAMPLE_2, RESISTOR_50K, FREQ_MODE_SPREAD);
Adafruit_FreeTouch touch_dec
        = Adafruit_FreeTouch(PIN_DEC, OVERSAMPLE_2, RESISTOR_50K, FREQ_MODE_SPREAD);

// Filtered touch values (0-1023)
uint16_t touch_value_power = 0;
uint16_t touch_value_inc = 0;
uint16_t touch_value_dec = 0;

// Touch pin states (on/off)
bool pinState_power = false;
bool pinState_inc = false;
bool pinState_dec = false;
bool pinState_power_previous = false;
bool pinState_inc_previous = false;
bool pinState_dec_previous = false;

bool ledOn = false;
uint8_t brightness = BRIGHTNESS_1;  // Set to default power on brightness

// The setup routine runs once when you press reset:
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("DIYson LED controller");

    pinMode(PIN_INDICATOR_LED, OUTPUT);

    // Turn off the LED
    analogWrite(PIN_DAC, 0);

    // Initialize Touch sensors
    if(!touch_power.begin()) {
        Serial.println("Failed to init touch_power");
    }
    if(!touch_inc.begin()) {
        Serial.println("Failed to init touch_inc");
    }
    if(!touch_dec.begin()) {
        Serial.println("Failed to init touch_dec");
    }
}

// The loop routine runs over and over again forever:
void loop() {
    updateTouchValues();
    updatePinStates();

    if(isRisingEdge(pinState_power, &pinState_power_previous)) {
        ledOn = !ledOn;
        Serial.print("LED toggle - ");
        Serial.println(ledOn);

        if(ledOn == true) {
            setLed((BRIGHTNESS_E)brightness);
        } else {
            setLed(BRIGHTNESS_OFF);
        }
    }

    if(ledOn == true) {
        if(isRisingEdge(pinState_inc, &pinState_inc_previous)) {
            if(brightness < BRIGHTNESS_MAX) {
                brightness++;
                Serial.print("Brightness (++): ");
                Serial.println(brightness);
                setLed((BRIGHTNESS_E)brightness);
            }
        }

        if(isRisingEdge(pinState_dec, &pinState_dec_previous)) {
            if(brightness > BRIGHTNESS_OFF) {
                brightness--;
                Serial.print("Brightness (--): ");
                Serial.println(brightness);
                setLed((BRIGHTNESS_E)brightness);
            }
        }
    }
}

// Calculate output value of EMA filter
// https://www.norwegiancreations.com/2015/10/tutorial-potentiometers-with-arduino-and-filtering/
// https://en.wikipedia.org/wiki/Exponential_smoothing
// https://youtu.be/1e_ZB8p5n6s
inline uint16_t filter_ema_calc(uint16_t x, uint16_t y_prev) {
    return FILTER_EMA_ALFA * x + (1 - FILTER_EMA_ALFA) * y_prev;
}

void updateTouchValues() {
    uint16_t touch_value_power_raw = 0;
    uint16_t touch_value_inc_raw = 0;
    uint16_t touch_value_dec_raw = 0;

    static uint16_t touch_value_power_previous = 0;
    static uint16_t touch_value_inc_previous = 0;
    static uint16_t touch_value_dec_previous = 0;

    touch_value_power_raw = touch_power.measure();
    touch_value_inc_raw = touch_inc.measure();
    touch_value_dec_raw = touch_dec.measure();

    touch_value_power = filter_ema_calc(touch_value_power_raw, touch_value_power_previous);
    touch_value_inc = filter_ema_calc(touch_value_inc_raw, touch_value_inc_previous);
    touch_value_dec = filter_ema_calc(touch_value_dec_raw, touch_value_dec_previous);

    // Store current y as previous value, needed for next cycle calculation
    touch_value_power_previous = touch_value_power;
    touch_value_inc_previous = touch_value_inc;
    touch_value_dec_previous = touch_value_dec;

#ifdef DEBUG_TOUCH_VALUES
    Serial.print(touch_value_power_raw);
    Serial.print(",");
    Serial.print(touch_value_power);
    Serial.print(",");

    Serial.print(touch_value_inc_raw);
    Serial.print(",");
    Serial.print(touch_value_inc);
    Serial.print(",");

    Serial.print(touch_value_dec_raw);
    Serial.print(",");
    Serial.print(touch_value_dec);
    Serial.print("\n");
#endif
}

inline bool isTouchPinPressedWithHysteresis(uint16_t measurement, bool previousState) {
    bool pinPressed = previousState;

    if(previousState == false) {
        if(measurement > TOUCH_PRESSED_THRESHOLD_HIGH) {
            pinPressed = true;
        }
    } else {
        // previousState == true
        if(measurement < TOUCH_PRESSED_THRESHOLD_LOW) {
            pinPressed = false;
        }
    }

    return pinPressed;
}

void updatePinStates() {
    pinState_power = isTouchPinPressedWithHysteresis(touch_value_power, pinState_power);
    pinState_inc = isTouchPinPressedWithHysteresis(touch_value_inc, pinState_inc);
    pinState_dec = isTouchPinPressedWithHysteresis(touch_value_dec, pinState_dec);

#ifdef DEBUG_PIN_STATES
    Serial.print(pinState_power);
    Serial.print(",");

    Serial.print(pinState_inc);
    Serial.print(",");

    Serial.print(pinState_dec);
    Serial.print("\n");
#endif
}

inline bool isRisingEdge(bool pinState_current, bool *pinState_previous) {
    bool returnState;

    if(pinState_current == true && *pinState_previous == false) {
        returnState = true;
    } else {
        returnState = false;
    }

    *pinState_previous = pinState_current;

    return returnState;
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

#ifdef DEBUG_INDICATOR_LED
    if(brightness == BRIGHTNESS_OFF) {
        digitalWrite(PIN_INDICATOR_LED, DEBUG_LED_ST_OFF);
    } else {
        digitalWrite(PIN_INDICATOR_LED, DEBUG_LED_ST_ON);
    }
#endif
}
