/**
 *
 * kaboom library for Arduino
 * URL...
 *
 * MIT License
 * (c) 2023 Aunders Hallsten
 *
**/

#ifndef KABOOM_h
#define KABOOM_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class kaboom
{
private:
    byte PD_SCK;     // Power Down and Serial Clock Input Pin
    byte DOUT;       // Serial Data Output Pin
    byte GAIN;       // amplification factor
    long OFFSET = 0; // used for tare weight
    float SCALE = 1; // used to return weight in grams, kg, ounces, whatever

public:
    kaboom();
    virtual ~kaboom();

    // read the analog pins that have been defined
    bool Touch_getXY(void);
    // create buttons with location, color, text, and text size
    void buttonInitialization();
    // draw buttons for numpad screen
    void drawNumberButtons();
    // call all functions necessary for tft to run
    void tftSetup();
    // call in loop to see if any buttons have been pressed
    void buttonChecks();
    // a way to read incoming data from serial port
    byte read_line(char *buffer, byte buffer_length);
    // call to tell user how to navigate the UI
    void displayMenu();
    // Call to display scales value
    void readScale();
    // Call to start scale on initial power on
    void scaleInitialize();
    // call to tie raw value to zero reading
    void tareScale();
    // Interactive calibration via serial port or touchscreen to set scaling based on any calibration weight
    void calibrateScale();
    // Interactive calibration via serial port or touchscreen to set scaling based on any calibration weight
    void calibrate_scale(void);
    // Start the stepper motors
    void stepperInitialize();
    // Run when delivering powder in an automated fasion
    void conveyKaboom();
};

#endif /* KABOOM_h */


