#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Pin definitions
#define BIT1                        (0x08)     // digital pin 8
#define BIT2                        (0x07)     // digital pin 7
#define BIT3                        (0x06)     // digital pin 6
#define BIT4                        (0x05)     // digital pin 5
#define BIT5                        (0x04)     // digital pin 4
#define STATUS_PIN                  (0x02)     // digital pin 2
#define BUTTON_PIN                  (0x0A)     // digital pin 10

// Other macros
#define BIT_COUNT                   (5)        // total number of bits
#define ACCEL_TRIGGER               (0)        // accelerometer trigger type
#define BUTTON_TRIGGER              (1)        // button trigger type
#define TRIGGER_INTERVAL            (50)       // trigger interval duration (in ms)
#define SERIAL_INTERVAL             (100)      // serial interval duration (in ms)
#define HOLD_INTERVAL               (5000)     // duration for a hold event (in ms)
#define LED_INTERVAL                (3000)     // duration LEDs are lit up on button press (in ms)
#define ACCEL_RES                   (10)       // acceleration measurement resolution (in ms/measurement)
#define MIN_MEAS                    (70)       // minimum value required to trigger measuring
#define DIFF_THRESHOLD              (0.15)     // Acceptable measurement error

// Assign an ID to the accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(13337);

// Counters
uint8_t bitArray[BIT_COUNT] = {BIT1, BIT2, BIT3, BIT4, BIT5};  // an array of LED bits in order
uint8_t elapsedSeconds;                        // the elapsed counting time (in s)
uint8_t measDiff;                              // int counter tracking measurement difference

// Trackers
bool buttonState;                              // boolean tracking button trigger state
bool lastButtonState;                          // boolean tracking last button trigger state
bool counting;                                 // boolean used to track whether timing is occuring
bool flightOver;                               // boolean used to track whether flight has ended
bool statusPinValue;                           // boolean used to track state of the status LED (on/off)
float accelArray[ACCEL_RES];                   // float array tracking accel measurements
unsigned long startTime;                       // the starting time for counting (in ms)
unsigned long elapsedTime;                     // the elapsed counting time (in ms)
unsigned long currentTriggerTime;              // the current trigger timer count (in ms)
unsigned long previousTriggerTime;             // the previous trigger timer count (in ms)
unsigned long currentSerialTime;               // the current serial timer count (in ms)
unsigned long previousSerialTime;              // the previous serial timer count (in ms)
unsigned long buttonHoldTime;                  // tracks how long button has been held
unsigned long ledTime;                         // tracks how long the status LED has been on

// The setup routine runs once at the start of operation.
void setup(void) {
    // Start serial communication
    Serial.begin(9600);
    Serial.println("GLIDER PROJECT\tSYSTEM READY");
    Serial.println("X:\t\tY:\t\tZ:\t\tElapsed Time:"); Serial.println("");  // display column headers

    // Initialise the accel
    if(!accel.begin()) {
      // Problem detecting accel
      Serial.println("ADXL345 not detected. Check connections!");
      while(1);                                              // hang the device
    }

    // Set the range
    accel.setRange(ADXL345_RANGE_16_G);                      // set accel range

    // Set the LED pins as outputs
    for (uint8_t i = 0; i < BIT_COUNT; ++i) {
        pinMode(bitArray[i], OUTPUT);
    }

    // Set the status LED pin as an output
    pinMode(STATUS_PIN, OUTPUT);

    // Enable the use of the button
    pinMode(BUTTON_PIN, INPUT);                              // set button pin as input
    digitalWrite(BUTTON_PIN, HIGH);                          /* turn on pullup resistors - wire button so
                                                                 a press shorts pin to ground */

    // Initialise some variables & clear the display
    lastButtonState = false;                                 // set last button state to be unpressed
    counting = false;                                        // begin operation without timing
    startTime = 0;                                           // set the starting time to 0
    elapsedTime = startTime;                                 // set the elapsed time to the start time
    previousTriggerTime = 0;                                 // set the trigger timer to 0
    previousSerialTime = 0;                                  // set the serial timer to 0
    buttonHoldTime = 0;                                      // set the button timer to 0
    ledTime = 0;                                             // set the status led timer to 0
    statusPinValue = false;                                  // set the status LED to off
    flightOver = false;                                      // set the flight tracker to false
    flash_display();                                         // flash the display
}

// The loop routine runs over and over again forever.
void loop(void) {
    // Get a new sensor event
    sensors_event_t event;                                   // initialise event struct
    accel.getEvent(&event);                                  // populate event struct
    report_to_serial(event);                                 // send data out to serial

    // Check for button press or timer trigger event
    buttonState = trigger_read(BUTTON_TRIGGER, event);       // store current button state
    if (buttonState == true && lastButtonState == false) {   // button down press
        lastButtonState = buttonState;                       // update button state
        buttonHoldTime = millis();                           // store the time in case of hold
        delay(5);                                            // short delay to debounce switch
    } else if (buttonState == true && lastButtonState == true) {  // button is held
        lastButtonState = buttonState;                       // update button state
        if (millis() - buttonHoldTime >= HOLD_INTERVAL) {    // button held for 5 seconds
            // Reset the system
            reset();                                         // call the reset function
        }
    } else if (buttonState == false && lastButtonState == true) {  // button release
        lastButtonState = buttonState;                       // update button state
        if (flightOver == false) {                           // if haven't launched
            display_time(bit(BIT_COUNT) - 1);                // light up all LEDs
            delay(LED_INTERVAL);                             // delay to display time
            clear_display();                                 // turn off all LEDs
        }

    } else {                                                 // no button press
        lastButtonState = buttonState;                       // update button state
    }

    // Handle timer
    if (launch_detected(event) || landing_detected(event)) { // if a timer event occurs
        if (counting == true) {                              // timer is counting - stop it
            elapsedTime = millis() - startTime;              // store elapsed time
            counting = false;                                // stop counting
            elapsedSeconds = elapsedTime / 1000;             // convert time to seconds

            // Report elapsed time
            Serial.print("\tFlight time: \t");
            Serial.print(elapsedSeconds);                    // print time
            Serial.println(" seconds.");
            if (!(display_time(elapsedSeconds))) {           // error displaying time
                Serial.print("Could not display time. ");
            }
        } else {                                             // timer is not counting - start it
            startTime = millis();                            // store the start time
            counting = true;                                 // start counting
        }
    }

    // Blink the status LED
    if (flightOver == false) { 								 // no longer ready to read
        if (counting) { 									 // if the timer is active
            if (millis() - ledTime >= 200) { 				 // toggle LED state every 200 ms
                ledTime = millis();
                digitalWrite(STATUS_PIN, statusPinValue);
                statusPinValue = !statusPinValue;
            }
        } else {
            digitalWrite(STATUS_PIN, HIGH); 				 // have not launched, enable pin to show ready state
        }
    } else {
      digitalWrite(STATUS_PIN, LOW); 						 // landed, turn off the status pin
    }
}

// Wrapper function that checks whether the timer should start.
bool launch_detected(sensors_event_t event) {
    // Check if it is an appropriate time to detect a launch
    if (counting == false && flightOver == false && trigger_read(ACCEL_TRIGGER, event)) {
        Serial.println("Go for launch!");                    // high acceleration is present
        return true;
    } else {
        return false;                                        // glider still idle
    }
}

// Function checks whether a landing event has occured.
bool landing_detected(sensors_event_t event) {
    if (counting == true) {                                  // check whether a launch has occurred and we are timing
        // Check whether the glider is not moving (easy check for a landing)
        for (uint8_t i = 0; i < ACCEL_RES; ++i) {            // populate the measurement array
            accelArray[i] = event.acceleration.y;            // set the element to the accel value
            delay(TRIGGER_INTERVAL);                         // wait for some time to pass

            // // Get a new sensor event
            accel.getEvent(&event);                          // populate event struct with accel values
        }

        // Check whether the measurements are the same (within an error margin)
        for (uint8_t i = 1; i < ACCEL_RES; ++i) {
            if (abs(accelArray[i] - accelArray[i - 1]) >= DIFF_THRESHOLD) {  /* if the absolute value of the difference
                                                                                 is within a margin of error*/
                Serial.println("We're still soaring.");      // glider hasn't landed yet
                return false;                                // landing has not been detected
            }
        }

        // Values are all similar - we have probably landed
        Serial.println("The seagull has landed.");           // output a confirmation of landing
        flightOver = true;                                   // flight over -> end flight tracking
        return true;                                         // landing has been detected
    } else {
        return false;                                        // have not yet launched
    }
}

// Nearly identical to the setup command. Sets variables and settings back to their initial states.
void reset(void) {
    // Ideally this would simply call the setup function, but is useful as a separate function
    // To play around with certain parameters after a reset.
    Serial.println("Resetting the device...");               // output a reset has taken place
    // Apply the same logic from the setup command
    Serial.println("GLIDER PROJECT\tSYSTEM READY");
    Serial.println("X:\t\tY:\t\tZ:\t\tElapsed Time:"); Serial.println("");  // display column headers

    // Initialise the accel
    if(!accel.begin())
    {
      // Problem detecting accel
      Serial.println("ADXL345 not detected. Check connections!");
      while(1);                                              // hang the device
    }

    // Set the range
    accel.setRange(ADXL345_RANGE_16_G);                      // set accel range

    // Set the LED pins as outputs
    for (uint8_t i = 0; i < BIT_COUNT; ++i) {
        pinMode(bitArray[i], OUTPUT);
    }

    // Set the status LED pin as an output
    pinMode(STATUS_PIN, OUTPUT);

    // Enable the use of the button
    pinMode(BUTTON_PIN, INPUT);                              // set button pin as input
    digitalWrite(BUTTON_PIN, HIGH);                          /* turn on pullup resistors - wire button so
                                                                 a press shorts pin to ground */

    // Initialise some variables & clear the display
    lastButtonState = false;                                 // set last button state to be unpressed
    counting = false;                                        // begin operation without timing
    startTime = 0;                                           // set the starting time to 0
    elapsedTime = startTime;                                 // set the elapsed time to the start time
    previousTriggerTime = 0;                                 // set the trigger timer to 0
    previousSerialTime = 0;                                  // set the serial timer to 0
    buttonHoldTime = 0;                                      // set the button timer to 0
    ledTime = 0;                                             // set the status led timer to 0
    statusPinValue = false;                                  // set the status LED to off
    flightOver = false;                                      // set the flight tracker to false
    flash_display();                                         // flash the display
}

// Outputs useful information to serial.
void report_to_serial(sensors_event_t event) {
    currentSerialTime = millis();                            // update timer

    if (currentSerialTime - previousSerialTime >= SERIAL_INTERVAL) {  // have we recently reported data?
        previousSerialTime = currentSerialTime;              // update last serial time

        // Display the results (acceleration is measured in m/s^2)
        Serial.print(event.acceleration.x); Serial.print("\t\t");
        Serial.print(event.acceleration.y); Serial.print("\t\t");
        Serial.print(event.acceleration.z); Serial.print("\t\t");
        Serial.print(elapsedTime); Serial.print("\t\t"); Serial.println("");
    }
}

// Returns a true on a triggered reading and false on an untriggered reading.
bool trigger_read(uint8_t triggerType, sensors_event_t event) {
    switch (triggerType) {
        case ACCEL_TRIGGER:                                  // accelerometer
            currentTriggerTime = millis();                   // update timer
            if (currentTriggerTime - previousTriggerTime >= TRIGGER_INTERVAL) {
                // Time to read data
                previousTriggerTime = currentTriggerTime;    // update timer
                if (event.acceleration.y >= MIN_MEAS) {
                    return true;
                }
            }
            return false;
        case BUTTON_TRIGGER:                                 // button
            if (digitalRead(BUTTON_PIN) == LOW) {            // button is pressed
                return true;
            } else {                                         // button not pressed
                return false;
            }
    }
}

// Flashes the display to show initialisation is complete.
void flash_display(void) {
    clear_display();                                         // clear the display

    // cycle up
    for (uint8_t i = 0; i < BIT_COUNT; ++i) {
        digitalWrite(bitArray[i], HIGH);
        delay(50);
        digitalWrite(bitArray[i], LOW);
    }

    // cycle down
    for (uint8_t i = 0; i <= BIT_COUNT; ++i) {
        digitalWrite(bitArray[BIT_COUNT - i], HIGH);
        delay(50);
        digitalWrite(bitArray[BIT_COUNT - i], LOW);
    }
}

// Wrapper function which clears the display, turning all LEDs off.
void clear_display(void) {
    display_time(0);
}

// Takes an unsigned 8-bit integer and displays it in binary using the mounted LEDs.
bool display_time(uint8_t number) {
    if (number < bit(BIT_COUNT)) {                           // number is valid (less than 31)
        for (uint8_t i = 0; i < BIT_COUNT; ++i) {            // iterate over each bit
            digitalWrite(bitArray[i], bitRead(number, i));   // set corresponding LED on/off
        }
        return true;                                         // executed successfully
    } else {                                                 // error - value will overflow
        return false;
    }
}
