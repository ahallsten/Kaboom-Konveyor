#include "HX711.h"
#include <AccelStepper.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <SPI.h>
#include <Wire.h>

// PIN SELECTION
// HX711 circuit wiring
#define LOADCELL_SCK_PIN 2
#define LOADCELL_DOUT_PIN 3

// STEPPER circuit wiring
// #define STOP 2
// #define AXISSELECT 3
#define DIR 6
#define STEP 4
#define SLEEP 5
// #define RESET 40
// #define FAULT 42

// Units of scale
#define UNITS_G 0
#define UNITS_GN 1

// VARIABLES
// STEPPER variables
int motor_interface_type = 1; // Type 1 is a Driver with 2 pins, STEP and DIR
long accel = 1200;
long max_speed = 1200;
long speed = 1000;
int pos = 0;
float mode_state = 1;
int mode_high_speed = 1000; // default for full step mode
byte tol = 5;
float minAccel = 0;
float maxAccel = 80000;
float minSpeed = 700;
float maxspeed = 5000;
float minPos = 0;
float maxPos = 5000;
long step = 0;

// HX711 variables
byte setting_sample_average = 10;
bool tare_button_state;
byte scale_init_count = 0;
byte scale_init_count_threshold = 20; // max loops to wait for scale settling
int threshold = 20;                   // value used to test when scale has settled
int amplitude;                        // difference between old and new readings.
int read_avg = 5;                     // number of average readings to take
byte count = 5;                       // quantity needed for determine stability of reading
unsigned long delay_ms = 20;
int init_retries = 5;
int powder_current;    // amount of powder in the tray currently
int powder_to_deliver; // Powder called for by user
int setting_tare_point;
int setting_average_amount;
byte setting_units; // Lbs or kg?
int setting_calibration_factor = 1000;

// Placeholder for serial communication
char inString;

// OBJECT INITIALIZATION
// Inititize an HX711 object
HX711 scale;

// Initialize a stepper and the pins it will use
AccelStepper stepper = AccelStepper(motor_interface_type, STEP, DIR); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// // Connect a stepper motor with 200 steps per revolution (1.8 degree)
// // to motor port #2 (M3 and M4)
// Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 2);

byte read_line(char *buffer, byte buffer_length)
{
  memset(buffer, 0, buffer_length); // Clear buffer

  byte read_length = 0;
  while (read_length < buffer_length - 1)
  {
    while (!Serial.available())
      ;
    byte c = Serial.read();

    if (c == 0x08 || c == 0x7f)
    { // Backspace characters
      if (read_length < 1)
        continue;

      --read_length;
      buffer[read_length] = '\0'; // Put a terminator on the string in case we are finished

      Serial.print((char)0x08); // Move back one space
      Serial.print(F(" "));     // Put a blank there to erase the letter from the terminal
      Serial.print((char)0x08); // Move back again

      continue;
    }

    Serial.print((char)c); // Echo the user's input

    if (c == '\r')
    {
      Serial.println();
      buffer[read_length] = '\0';
      break;
    }
    else if (c == '\n')
    {
    }
    else
    {
      buffer[read_length] = c;
      ++read_length;
    }
  }

  return read_length;
}

void displayMenu()
{
  Serial.println();
  Serial.println(F("Main Menu:"));
  Serial.println(F("1) Convey Kaboom"));
  Serial.println(F("2) Tare Scale"));
  Serial.println(F("3) Calibrate Scale"));
  Serial.println(F("4) Read Scale Continuously"));
  Serial.println(F("5) Reserved"));
  Serial.println(F("x) Exit"));
}

void readCommand()
{
}

void readScale()
{
  while (!Serial.available())
  {
    Serial.println(scale.read_average(read_avg));
  }
  Serial.println("quiting scale reading...");
  displayMenu();
}

void scaleInitialize()
{
  Serial.println("Initializing Scale on pins:");
  Serial.print("SDA: ");
  Serial.println(LOADCELL_DOUT_PIN);
  Serial.print("CLK: ");
  Serial.println(LOADCELL_SCK_PIN);

  // if (scale.wait_ready_retry(init_retries, delay_ms))
  // {
  //   long reading = scale.read();
  //   Serial.print("HX711 initial reading: ");
  //   Serial.println(reading);
  // }
  // else
  // {
  //   Serial.println("HX711 not found.");
  // }

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // while (amplitude >= threshold) // test and keep testing until scale is settled and ready for tare.
  // {
  //   int old_scale_reading = 0;
  //   int new_scale_reading = scale.read();
  //   int amplitude = new_scale_reading - old_scale_reading;

  //   scale_init_count++;
  //   if (scale_init_count >= scale_init_count_threshold)
  //   {
  //     Serial.print("scale not settling count:");
  //     Serial.println(scale_init_count);
  //     break;
  //   }
  // }
}

void tareScale()
{
  Serial.print(F("\n\rGetting Tare point: "));
  scale.tare();                                // Reset the scale to 0
  setting_tare_point = scale.read_average(10); // Get 10 readings from the HX711 and average them
  Serial.println(setting_tare_point);

  Serial.println("tareing scale...");
  if (scale.wait_ready_retry(10))
  {
    scale.tare(); // reset the scale to 0

    Serial.print("raw reading: \t\t");
    Serial.println(scale.read()); // print a raw reading from the ADC

    Serial.print("Average Reading: \t\t");
    Serial.println(scale.read_average(read_avg)); // print the average of 20 readings from the ADC

    Serial.print("get value: \t\t");
    Serial.println(scale.get_value(5)); // print the average of 5 readings from the ADC minus the tare weight, set with tare()

    Serial.print("get units: \t\t");
    Serial.println(scale.get_units(5), 1); // print the average of 5 readings from the ADC minus tare weight, divided
    Serial.println("done tareing");
    displayMenu();
  }
  else
  {
    Serial.println("HX711 not found.");
  }
}

void calibrateScale()
{
  Serial.println("Put the 50g weight on the scale and enter serial");
  Serial.println("calibrating...");
  while (!Serial.available())
  {
    scale.set_scale(50.f);
  }
  Serial.println("done calibrating, offset: ");
  Serial.println(scale.get_offset());
  Serial.println("units:");
  Serial.println(scale.get_units());
}

void calibrate_scale(void)
{
  Serial.println();
  Serial.println();
  Serial.println(F("Scale calibration"));
  Serial.println(F("Place known weight on scale. Press a key when weight is in place and stable."));

  while (Serial.available() == false)
    ; // Wait for user to press key

  Serial.print(F("Tare: "));
  Serial.println(setting_tare_point);

  long rawReading = scale.read_average(setting_average_amount); // Take average reading over a given number of times
  Serial.print(F("Raw: "));
  Serial.println(rawReading);

  Serial.print(F("Current Reading: "));
  Serial.print(scale.get_units(setting_average_amount), 4); // Show 4 decimals during calibration
  if (setting_units == UNITS_G)
    Serial.print(F("lbs"));
  if (setting_units == UNITS_GN)
    Serial.print(F("kg"));
  Serial.println();

  Serial.print(F("Calibration Factor: "));
  Serial.print(setting_calibration_factor);
  Serial.println();

  while (Serial.available())
    Serial.read(); // Clear anything in RX buffer

  Serial.print(F("Please enter the weight currently sitting on the scale: "));

  // Read user input
  char newSetting[15]; // Max 15 characters: "12.5765" = 8 characters (includes trailing /0)
  read_line(newSetting, sizeof(newSetting));

  float weightOnScale = atof(newSetting); // Convert this string to a float
  Serial.println();

  Serial.print(F("User entered: "));
  Serial.println(weightOnScale, 4);

  // Convert this weight to a calibration factor

  // tare: 210193
  // raw: 246177
  // User Input: 0.5276 kg
  // avg: 4 times

  // get_units = (raw-OFFSET) / calibration_factor
  // 0.5276 = (246177-210193) / cal_factor
  // 114185 / .45 = 256744

  setting_calibration_factor = (rawReading - setting_tare_point) / weightOnScale;

  Serial.print(F("New Calibration Factor: "));
  Serial.print(setting_calibration_factor);
  Serial.println();

  scale.set_scale(setting_calibration_factor); // Go to this new cal factor

  // Record this new value to EEPROM
  // record_system_settings();

  Serial.print(F("New Scale Reading: "));
  Serial.print(scale.get_units(setting_average_amount), 4); // Show 4 decimals during calibration
  Serial.print(F(" "));
  if (setting_units == UNITS_G)
    Serial.print(F("lbs"));
  if (setting_units == UNITS_GN)
    Serial.print(F("kg"));
  Serial.println();
}

void stepperInitialize()
{
  stepper.setMaxSpeed(max_speed);
  stepper.setSpeed(speed);
  stepper.setAcceleration(accel);
}

void conveyKaboom()
{

  while (powder_current <= powder_to_deliver)
  {
    if (Serial.available())
    {
      // Read command
      inString = Serial.read();
      Serial.print("recieved: ");
      Serial.println(inString);
      Serial.print("exiting kaboom conveyance");
    }
    step = 1;
    stepper.move(step);
    stepper.run();
    powder_current = scale.read();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Kaboom Konveyor V0.1");
  stepperInitialize();
  scaleInitialize();
  displayMenu();
}

void loop()
{
  if (Serial.available())
  {
    // Read command
    inString = Serial.read();
    Serial.print("recieved: ");
    Serial.println(inString);

    // Execute command
    if (inString == '1')
    {
      conveyKaboom();
    }
    else if (inString == '2')
    {
      tareScale();
    }
    else if (inString == '3')
    {
      calibrateScale();
    }
    else if (inString == '4')
    {
      readScale();
    }
    else if (inString == '5')
    {
      delay(500);
    }
    else if (inString == 'x')
    {
      // Do nothing, just exit
      Serial.println(F("Exited System Config, press any key to go back into System Config"));
    }
  }
}

//-------------------------------------------------------------------------------------------------
// FOR STEPPER MOTORS
// Run to some positions
// if (Serial.available() > 1)
// {
//   mode_state = !mode_state;
// }
// stepper.runToNewPosition(0);
// stepper.runToNewPosition(5000);
// stepper.runToNewPosition(1000);
// stepper.runToNewPosition(1200);

//-------------------------------------------------------------------------------------------------
// FOR SCALE
// tare_button_state = digitalRead(TARE_BUTTON_PIN);