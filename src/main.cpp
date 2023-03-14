/*Libraries for Load Cell Amp*/
#include "HX711.h"
#include <SPI.h>
#include <Wire.h>
/*Libraries for Stepper Driver*/
#include <AccelStepper.h>
#include <Servo.h>
/*Libraries for 3.5TFT*/
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
// #include "kaboom.h"

/*PIN SELECTIONS AND DEFINITIONS*/
// HX711 circuit wiring
#define LOADCELL_SCK_PIN 21
#define LOADCELL_DOUT_PIN 20
// STEPPER circuit wiring
// #define STOP 2
// #define AXISSELECT 3
#define DIR 6
#define STEP 4
#define SLEEP 5
// #define RESET 40
// #define FAULT 42
// Units of Scale
#define UNITS_G 0
#define UNITS_GN 1

// Screen Analog Values
#define MINPRESSURE 200
#define MAXPRESSURE 1000
// Screen Colors
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define RED 0x07FF
#define GREEN 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

/*VARIABLES*/
// HX711 variables
byte setting_sample_average = 10;
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
float raw_reading;
float scale_reading;
float cal_factor;
long tare_point;
float cal_weight = 50.43; // 50g cal weight
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
// TFT variables
const int XP = 7, XM = A1, YP = A2, YM = 6; // 320x480 ID=0x6814
const int TS_LEFT = 176, TS_RT = 921, TS_TOP = 177, TS_BOT = 939;
uint16_t ID;
uint16_t x0 = 0; // location for inputs
uint16_t y0 = 20;
uint16_t x1 = 60; // location for inputs
uint16_t y1 = 60;
uint16_t x2 = 60; // location for inputs
uint16_t y2 = 90;
uint16_t x3 = 60; // location for inputs
uint16_t y3 = 120;
uint16_t x4 = 60; // location for inputs
uint16_t y4 = 150;
uint16_t x5 = 60; // location for inputs
uint16_t y5 = 180;
uint16_t x6 = 60; // location for inputs
uint16_t y6 = 210;
char buf[11];         // buffer for number inputs
char buf2[11];        // buffer for number inputs
int pixel_x, pixel_y; // Touch_getXY() updates global vars
// Serial communication variables
char inString;
bool finish = 1;
bool home = 1;

/*OBJECT INITIALIZATION*/
// Initialize an object for the HX711 called "scale"
HX711 scale;
// Initialize an object for the stepper called "stepper" and the pins it will use
AccelStepper stepper = AccelStepper(motor_interface_type, STEP, DIR); // Defaults to AccelStepper::FULL4WIRE (4 pinsSerial) on 2, 3, 4, 5
// Initialize an object for the screen called "tft" for graphics
MCUFRIEND_kbv tft;
// Initialize an object for the screen called "ts" for touch
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
// Initialize button objects for UI
Adafruit_GFX_Button one_btn, two_btn, three_btn, four_btn, five_btn, six_btn, seven_btn, eight_btn, nine_btn, zero_btn, back_btn, fwd_btn, up_btn, down_btn, conveyKaboom_btn, other_btn, scaleRead_btn, testMode_btn, home_btn, tare_ready_btn, calibrate_btn;

/*HOME BREW FUNCTIONS*/
bool Touch_getXY(void)
{
  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT); // restore shared pins
  pinMode(XM, OUTPUT);
  digitalWrite(YP, HIGH); // because TFT control pins
  digitalWrite(XM, HIGH);
  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (pressed)
  {
    pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width()); //.kbv makes sense to me
    pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
  }
  return pressed;
}

void buttonInitialization() // create buttons with location, color, text, and text size
{
  one_btn.initButton(&tft, 60, 120, 80, 80, YELLOW, GREEN, RED, "1", 3);
  two_btn.initButton(&tft, 160, 120, 80, 80, YELLOW, GREEN, RED, "2", 3);
  three_btn.initButton(&tft, 260, 120, 80, 80, YELLOW, GREEN, RED, "3", 3);
  four_btn.initButton(&tft, 60, 220, 80, 80, YELLOW, GREEN, RED, "4", 3);
  five_btn.initButton(&tft, 160, 220, 80, 80, YELLOW, GREEN, RED, "5", 3);
  six_btn.initButton(&tft, 260, 220, 80, 80, YELLOW, GREEN, RED, "6", 3);
  seven_btn.initButton(&tft, 60, 320, 80, 80, YELLOW, GREEN, RED, "7", 3);
  eight_btn.initButton(&tft, 160, 320, 80, 80, YELLOW, GREEN, RED, "8", 3);
  nine_btn.initButton(&tft, 260, 320, 80, 80, YELLOW, GREEN, RED, "9", 3);
  zero_btn.initButton(&tft, 160, 420, 80, 80, YELLOW, GREEN, RED, "0", 3);

  conveyKaboom_btn.initButton(&tft, 160, 120, 280, 80, YELLOW, GREEN, RED, "Convey \n Kaboom", 2);
  other_btn.initButton(&tft, 160, 220, 280, 80, YELLOW, GREEN, RED, "Other", 3);
  scaleRead_btn.initButton(&tft, 160, 320, 280, 80, YELLOW, GREEN, RED, "Scale Raw Read", 3);
  testMode_btn.initButton(&tft, 160, 420, 280, 80, YELLOW, GREEN, RED, "Test Mode", 3);

  back_btn.initButton(&tft, 60, 420, 80, 80, YELLOW, GREEN, RED, "<-", 3); // work on positions
  fwd_btn.initButton(&tft, 260, 420, 80, 80, YELLOW, GREEN, RED, "->", 3); 
  up_btn.initButton(&tft, 60, 420, 80, 80, YELLOW, GREEN, RED, "up", 3);
  down_btn.initButton(&tft, 260, 420, 80, 80, YELLOW, GREEN, RED, "dwn", 3);

  tare_ready_btn.initButton(&tft, 160, 320, 280, 80, YELLOW, GREEN, RED, "Tare", 3);
  home_btn.initButton(&tft, 160, 420, 280, 80, YELLOW, GREEN, RED, "Home", 3);
  calibrate_btn.initButton(&tft, 160, 220, 280, 80, YELLOW, GREEN, RED, "Calibrate", 3);
}

void tftInitialize()
{
  ID = tft.readID();
  tft.begin(ID);
  tft.setRotation(0); // PORTRAIT
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, y0);
  sprintf(buf, "%-10s", "Kaboom Konveyor V0.3");
  tft.print(buf);
  buttonInitialization();
}

void drawMainMenuButtons()
{
  conveyKaboom_btn.drawButton(false);
  other_btn.drawButton(false);
  scaleRead_btn.drawButton(false);
  testMode_btn.drawButton(false);
}

void drawNumberButtons()
{
  one_btn.drawButton(false);
  two_btn.drawButton(false);
  three_btn.drawButton(false);
  four_btn.drawButton(false);
  five_btn.drawButton(false);
  six_btn.drawButton(false);
  seven_btn.drawButton(false);
  eight_btn.drawButton(false);
  nine_btn.drawButton(false);
  zero_btn.drawButton(false);
  back_btn.drawButton(false);
  fwd_btn.drawButton(false);
}

void drawScaleReadButtons()
{
  home_btn.drawButton();
  tare_ready_btn.drawButton();
  calibrate_btn.drawButton();
}

byte read_line(char *buffer, byte buffer_length) // a way to read incoming data from serial port
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

void displayMenu() // call to tell user how to navigate the UI
{
  drawMainMenuButtons();
  // Serial.println();
  // Serial.println(F("Main Menu:"));
  // Serial.println(F("1) Convey Kaboom"));
  // Serial.println(F("2) Tare Scale"));
  // Serial.println(F("3) Calibrate Scale"));
  // Serial.println(F("4) Read Scale Continuously"));
  // Serial.println(F("5) Reserved"));
  // Serial.println(F("x) finish"));
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
  // Wait for button press from screen. Ask Ready to start?
  while (powder_current <= powder_to_deliver)
  {
    if (Serial.available())
    {
      // Read command
      inString = Serial.read();
      Serial.print("recieved: ");
      Serial.println(inString);
      Serial.print("finishing kaboom conveyance");
    }
    step = 1;
    stepper.move(step);
    stepper.run();
    powder_current = scale.read();
  }
}

bool readScaleButtonChecks()
{
  bool down = Touch_getXY(); // need to find out if I need this
  home_btn.press(down && home_btn.contains(pixel_x, pixel_y));
  tare_ready_btn.press(down && tare_ready_btn.contains(pixel_x, pixel_y));
  calibrate_btn.press(down && calibrate_btn.contains(pixel_x, pixel_y));

  if (home_btn.justReleased())
    home_btn.drawButton();
  if (tare_ready_btn.justReleased())
    tare_ready_btn.drawButton();
  if (calibrate_btn.justReleased())
    calibrate_btn.drawButton();

  if (home_btn.justPressed())
  {
    home_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "going home");
    tft.setCursor(x0, y0);
    tft.print(buf);
    home = !home;
  }
  if (tare_ready_btn.justPressed())
  {
    tare_ready_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "tareing scale");
    tft.setCursor(x1, y1);
    tft.print(buf);
    finish = !finish;
  }
  if (calibrate_btn.justPressed())
  {
    calibrate_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "Calibrating   ");
    tft.setCursor(x1, y1);
    tft.print(buf);
    finish = !finish;
  }
  return finish;
}

void readScale()
{
  tft.fillScreen(BLACK);
  sprintf(buf, "%14s", "tare when ready");
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x0, y0);
  tft.print(buf);

  sprintf(buf, "%14s", "raw reading:");
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x1, y1);
  tft.print(buf);
  drawScaleReadButtons();

  while (finish) // FIRST TARE LOOP, print the raw reading and check for user to press tare or home !tare_ready_btn.justPressed() | !home_btn.justPressed()
  {
    raw_reading = scale.read_average(read_avg);
    sprintf(buf, "%10.2f", raw_reading);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    tft.setCursor(x2, y2);
    tft.print(raw_reading, 2);
    finish = readScaleButtonChecks();
  }
  finish = !finish; // toggle finish back
  sprintf(buf, "%14s", "tare offset:");
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x3, y3);
  tft.print(buf);
  scale.tare();
  scale.set_scale();
  tare_point = scale.get_offset();
  delay(500);

  sprintf(buf2, "%14d", tare_point); // print the offset to get a zero scale reading
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x4, y4);
  tft.print(buf2);

  tft.fillRect(0, y0, 320, 30, BLACK);    // erase raw reading: indicator
  sprintf(buf, "%14s", "cal when ready"); // replace with raw/w/tare: indicator
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x0, y0);
  tft.print(buf);

  tft.fillRect(0, y1, 320, 30, BLACK); // erase raw reading: indicator
  sprintf(buf, "%14s", "raw w/tare:"); // replace with raw/w/tare: indicator
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x1, y1);
  tft.print(buf);

  while (finish) // SECOND CALIBRATE LOOP, print the raw reading and wait for user to press calibrate or home
  {
    scale_reading = scale.get_value(read_avg);
    sprintf(buf, "%10.2f", scale_reading);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    tft.setCursor(x2, y2);
    tft.print(scale_reading, 2);
    finish = readScaleButtonChecks();
  }
  finish = !finish; // toggle finish back

  tft.fillRect(0, y3, 320, 30, BLACK); // erase tare offset: indicator
  sprintf(buf, "%14s", "cal factor:"); // replace with cal factor: indicator
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x3, y3);
  tft.print(buf);
  cal_factor = (scale_reading - raw_reading) / cal_weight;
  scale.set_scale(cal_factor);
  delay(500);

  sprintf(buf, "%14d", scale.get_scale()); // print the cal factor reading
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x4, y4);
  tft.print(buf);

  tft.fillRect(0, y0, 320, 30, BLACK);      // erase raw reading: indicator
  sprintf(buf, "%14s", "read or go home:"); // replace with raw/w/tare: indicator
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x1, y1);
  tft.print(buf);

  tft.fillRect(0, y1, 320, 30, BLACK);    // erase raw w/tare: indicator
  sprintf(buf, "%14s", "real readout: "); // replace with real reading
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(3);
  tft.setCursor(x1, y1);
  tft.print(buf);

  tft.fillRect(0, y2, 320, 30, BLACK); // erase scale_reading readout

  while (finish) // THRID SET SCALE CALIBRATION LOOP, print the scale reading and check for user to press calibrate or home
  {
    scale_reading = scale.get_units(read_avg);
    sprintf(buf, "%10.2f", scale_reading);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    tft.setCursor(x2, y2);
    tft.print(scale_reading, 2);
    finish = readScaleButtonChecks();
  }
}

bool testModeButtonChecks()
{
  bool down = Touch_getXY(); // need to find out if I need this
  home_btn.press(down && home_btn.contains(pixel_x, pixel_y));
  fwd_btn.press(down && fwd_btn.contains(pixel_x, pixel_y));
  back_btn.press(down && back_btn.contains(pixel_x, pixel_y));

  if (home_btn.justReleased())
    home_btn.drawButton();
  if (tare_ready_btn.justReleased())
    tare_ready_btn.drawButton();
  if (calibrate_btn.justReleased())
    calibrate_btn.drawButton();

  if (home_btn.justPressed())
  {
    home_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "going home");
    tft.setCursor(x0, y0);
    tft.print(buf);
    home = !home;
  }
  if (tare_ready_btn.justPressed())
  {
    tare_ready_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "tareing scale");
    tft.setCursor(x1, y1);
    tft.print(buf);
    finish = !finish;
  }
  if (calibrate_btn.justPressed())
  {
    calibrate_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "Calibrating   ");
    tft.setCursor(x1, y1);
    tft.print(buf);
    finish = !finish;
  }
  return finish;
}

void testMode()
{
}

void mainMenuButtonChecks() // Check buttons if any has been pressed
{
  // while(1) // loop until fwd button is pressed
  // {
  bool down = Touch_getXY();
  // Function to ou
  conveyKaboom_btn.press(down && conveyKaboom_btn.contains(pixel_x, pixel_y));
  calibrate_btn.press(down && calibrate_btn.contains(pixel_x, pixel_y));
  scaleRead_btn.press(down && scaleRead_btn.contains(pixel_x, pixel_y));
  testMode_btn.press(down && testMode_btn.contains(pixel_x, pixel_y));

  if (conveyKaboom_btn.justReleased())
    conveyKaboom_btn.drawButton();
  if (calibrate_btn.justReleased())
    calibrate_btn.drawButton();
  if (scaleRead_btn.justReleased())
    scaleRead_btn.drawButton();
  if (testMode_btn.justReleased())
    testMode_btn.drawButton();

  if (conveyKaboom_btn.justPressed())
  {
    conveyKaboom_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "kaboom");
    tft.setCursor(x0, y0);
    tft.print(buf);
    conveyKaboom();
  }
  if (other_btn.justPressed())
  {
    other_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "Calibrate");
    tft.setCursor(x0, y0);
    tft.print(buf);
    calibrateScale();
  }
  if (scaleRead_btn.justPressed())
  {
    scaleRead_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "Read Scale");
    tft.setCursor(x0, y0);
    tft.print(buf);
    readScale();
  }
  if (testMode_btn.justPressed())
  {
    testMode_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "Test Mode");
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
}

void numberInputButtonChecks()
{
  bool down = Touch_getXY();
  one_btn.press(down && one_btn.contains(pixel_x, pixel_y));
  two_btn.press(down && two_btn.contains(pixel_x, pixel_y));
  three_btn.press(down && three_btn.contains(pixel_x, pixel_y));
  four_btn.press(down && four_btn.contains(pixel_x, pixel_y));
  five_btn.press(down && five_btn.contains(pixel_x, pixel_y));
  six_btn.press(down && six_btn.contains(pixel_x, pixel_y));
  seven_btn.press(down && seven_btn.contains(pixel_x, pixel_y));
  eight_btn.press(down && eight_btn.contains(pixel_x, pixel_y));
  nine_btn.press(down && nine_btn.contains(pixel_x, pixel_y));
  zero_btn.press(down && zero_btn.contains(pixel_x, pixel_y));
  back_btn.press(down && back_btn.contains(pixel_x, pixel_y));
  fwd_btn.press(down && fwd_btn.contains(pixel_x, pixel_y));

  if (one_btn.justReleased())
    one_btn.drawButton();
  if (two_btn.justReleased())
    two_btn.drawButton();
  if (three_btn.justReleased())
    three_btn.drawButton();
  if (four_btn.justReleased())
    four_btn.drawButton();
  if (five_btn.justReleased())
    five_btn.drawButton();
  if (six_btn.justReleased())
    six_btn.drawButton();
  if (seven_btn.justReleased())
    seven_btn.drawButton();
  if (eight_btn.justReleased())
    eight_btn.drawButton();
  if (nine_btn.justReleased())
    nine_btn.drawButton();
  if (zero_btn.justReleased())
    zero_btn.drawButton();
  if (back_btn.justReleased())
    back_btn.drawButton();
  if (fwd_btn.justReleased())
    fwd_btn.drawButton();

  if (one_btn.justPressed())
  {
    one_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 1);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (two_btn.justPressed())
  {
    two_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 2);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (three_btn.justPressed())
  {
    three_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 3);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (four_btn.justPressed())
  {
    four_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 4);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (five_btn.justPressed())
  {
    five_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 5);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (six_btn.justPressed())
  {
    six_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 6);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (seven_btn.justPressed())
  {
    seven_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 7);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (eight_btn.justPressed())
  {
    eight_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 8);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (nine_btn.justPressed())
  {
    nine_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 9);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (zero_btn.justPressed())
  {
    zero_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10d", 0);
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (back_btn.justPressed())
  {
    back_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "backspace");
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
  if (fwd_btn.justPressed())
  {
    fwd_btn.drawButton(true);
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    sprintf(buf, "%10s", "forward");
    tft.setCursor(x0, y0);
    tft.print(buf);
  }
}

/*MAIN SETUP AND LOOP*/
void setup()
{
  Serial.begin(115200);
  Serial.println("Kaboom Konveyor V0.2");
  stepperInitialize();
  scaleInitialize();
  tftInitialize();
  displayMenu();
}

void loop()
{
  if (!finish)
  {
    finish = !finish;
    tft.fillScreen(BLACK);
  }
  mainMenuButtonChecks();
  // if (Serial.available())
  // {
  //   // Read command
  //   inString = Serial.read();
  //   Serial.print("recieved: ");
  //   Serial.println(inString);
  //   // Execute command
  //   if (inString == '1')
  //   {
  //     conveyKaboom();
  //   }
  //   else if (inString == '2')
  //   {
  //     tareScale();
  //   }
  //   else if (inString == '3')
  //   {
  //     calibrateScale();
  //   }
  //   else if (inString == '4')
  //   {
  //     readScale();
  //   }
  //   else if (inString == '5')
  //   {
  //     delay(500);
  //   }
  //   else if (inString == 'x')
  //   {
  //     // Do nothing, just finish
  //     Serial.println(F("finished System Config, press any key to go back into System Config"));
  //   }
  // }
}