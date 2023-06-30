// Project Rainfall
//  * Code for a module to measure rainfall as well as other useful items
//  * Designed for NodeMCU. Customizable additional sensors, the main one being rainfall sensor, 
//    which includes both a liquid level tape (https://www.adafruit.com/product/3828) and a 
//    solenoid (https://www.adafruit.com/product/996).
//  * Designed for external E-Ink display (https://www.adafruit.com/product/4197) and blynk
//    dashboard (https://blynk.io/) for measurement readouts.
//  * Template parts included for potential future sensor additions (OTHER_SENSOR)
//
// Some code is included from Adafruit examples, so the following is included
//   ***************************************************
//    Adafruit invests time and resources providing this open source code,
//    please support Adafruit and open-source hardware by purchasing
//    products from Adafruit!
//
//    Written by Limor Fried/Ladyada for Adafruit Industries.
//    MIT license, all text above must be included in any redistribution
//   ****************************************************
//
// Code started by Aidan St. George <aidanstgeorge1@gmail.com>

#include <Blynk.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_ThinkInk.h>
#include <SPI.h>

#define SD3   10

// CONFIGURATION VARIABLES
// set to true if sensor is being used, false otherwise
#define RAINFALL_SENSOR             true    // Liquid level
#define AEROSOL_SENSOR              false   // temp, pres, humidity
#define ATMOSPHERE_SENSOR           false
#define OTHER_SENSOR                false   // template

#define DEBUG                       true    // for Serial debug output
#define WIFI                        false
#define BLYNK                       false   // requires wifi
#define EINK_DISPLAY                true

#define BLYNK_TEMPLATE_ID           "TMPLxTz_BiVA"
#define BLYNK_DEVICE_NAME           "NodeMCU"
#define BLYNK_AUTH_TOKEN            "auth"
#define WIFI_NAME                   "ssid"
#define WIFI_PASS                   "pass"

#define BLYNK_UPDATE_INT            30000L  // milliseconds
#define EINK_UPDATE_INT             30000L  // milliseconds

#define RAINFALL_RESISTOR           560     // omhs
#define RAINFALL_FLUSH_LEVEL        1.0     // inches
#define RAINFALL_FLUSH_TIME         2000    // milliseconds
#define RF_MEASURE_INTERVAL         86400   // seconds
#define RAINFALL_CAL_M              -0.001001   // Calibration constants for converting raw analog input data to
#define RAINFALL_CAL_B              1.901       // inches of rain

#define RAINFALL_SENSOR_PIN         A0
#define SOLENOID_CONTROL_PIN        D0
#define DISPLAY_ECS_PIN             D4
#define DISPLAY_DC_PIN              D3
#define DISPLAY_SRCS                -1      // not used
#define DISPLAY_RST                 D8
#define DISPLAY_BUSY                D1 
#define DISPLAY_ENA                 SD3

// OTHER VARIABLES
// Do not change!

BlynkTimer timer;
uint64_t solenoid_timer;
bool do_display_update;

// E-ink display for 2.13" Monochrome e-ink display w/ SSD1680 chipset
ThinkInk_213_Mono_B74 display(DISPLAY_DC_PIN, DISPLAY_RST, DISPLAY_ECS_PIN, DISPLAY_SRCS, DISPLAY_BUSY);

/*
 * SETUP
 */
void setup() {
  // Serial
  if (DEBUG) {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Serial connected");
    Serial.println("Begin setup");
  }

  if (WIFI) {
    if (DEBUG) Serial.print("Connecting to wifi... ");
    WiFi.begin(WIFI_NAME, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      if (DEBUG) Serial.println("Connecting to WiFi...");
    }
    if (DEBUG) Serial.println( "connection successful!");
  }

  if (RAINFALL_SENSOR) {
    if (DEBUG) Serial.print("initializing rainfall sensor... ");
    
    pinMode(RAINFALL_SENSOR_PIN, INPUT);
    pinMode(SOLENOID_CONTROL_PIN, OUTPUT);
    solenoid_timer = millis();
    
    if (DEBUG) Serial.println("initialized");
  }

  if (AEROSOL_SENSOR) {
    if (DEBUG) Serial.print("initializing aerosol sensor... ");

    if (DEBUG) Serial.println("initialized");
  }

  if (ATMOSPHERE_SENSOR) {
    if (DEBUG) Serial.print("initializing atmosphere sensor... ");

    if (DEBUG) Serial.println("initialized");
  }

  if (OTHER_SENSOR) {

  }


  if (EINK_DISPLAY) {
    if (DEBUG) Serial.print("Initializing e-ink display... ");

    // Initialize SPI for display
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Pin that controls whether display draws power
    pinMode(DISPLAY_ENA, OUTPUT);
    digitalWrite(DISPLAY_ENA, LOW); // turn off

    display.begin(THINKINK_MONO);
    do_display_update = true;
    timer.setInterval(EINK_UPDATE_INT, eink_display_update);

    if (DEBUG) Serial.println("initialized");
  }

  if (BLYNK) {
    if (DEBUG) Serial.print("Connecting to blynk...");
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_NAME, WIFI_PASS);
    timer.setInterval(1000L, blynk_update);
    if (DEBUG) Serial.println("connected");
  }

  if (DEBUG) Serial.println("\nSetup successful\n\n");

  timer.setInterval(1000L, measure_rainfall); //TODO remove

  if (EINK_DISPLAY) eink_display_update();
}

/* 
 * Send updates to Blynk. This should only be called as a timer event, otherwise Blynk will get
 * overloaded and block this device.
 */
void blynk_update() {
  if (DEBUG) Serial.println("Updating Blynk");

  if (RAINFALL_SENSOR) {
    float rainfall = measure_rainfall();
    Blynk.virtualWrite(V4, rainfall);
  }
  
  if (AEROSOL_SENSOR) {

  }

  if (ATMOSPHERE_SENSOR) {

  }

  if (OTHER_SENSOR) {

  }
}


/*
 * MAINLOOP
 * Don't continuously check things! 
 * Only send updates to blynk on timed update.
 */
void loop() {
  // Don't continuously check things! 
  // Only send updates to blynk on timed update.
  if (is_button_pressed()) {
    do_display_update = true;
  }

  Blynk.run();
  timer.run();
}

bool is_button_pressed() {
  // Potential addition so display only updates on button press
  return true;
}

void eink_display_update() {
  if (!do_display_update) {
    return;
  }

  if (DEBUG) Serial.println("Updating display");

  digitalWrite(DISPLAY_ENA, HIGH);
  display.clearBuffer();
  display.setTextSize(2);
  display.setCursor(15, 0);
  display.setTextColor(EPD_BLACK);
  display.print(":Project Rainfall:");

  int current_line = 30;
  display.setTextSize(1);
  if (RAINFALL_SENSOR) {
    float rainfall = measure_rainfall();
    display.setCursor(0, current_line);
    display.print("Rainfall (in): " + String(rainfall));
    current_line += 12;
  }

  if (AEROSOL_SENSOR) {
    float aerosol = measure_aerosol();
    display.setCursor(0, current_line);
    display.print("Other measurement (dB/Hz): " + String(aerosol));
    current_line += 12;
  }

  if (ATMOSPHERE_SENSOR) {
    float atmosphere = measure_atmosphere();
    display.setCursor(0, current_line);
    display.print("Atmosphere: " + String(atmosphere));
    current_line += 12;
  }

  if (OTHER_SENSOR) {

  }

  display.display();
  digitalWrite(DISPLAY_ENA, LOW);
  do_display_update = false;
}

/*
 * MEASUREMENT FUNCTIONS
 * Rainfall function also needs to handle flushing water collection tube via solenoid
 */
float measure_rainfall() {
  if (DEBUG) Serial.println("Measuring");
  float reading = analogRead(RAINFALL_SENSOR_PIN);
 
  // convert the value to resistance
  reading = (1023 / reading)  - 1;
  reading = RAINFALL_RESISTOR / reading;

  // Convert to inches, values based on experimental calibration data and 5" liquid level tape
  reading = RAINFALL_CAL_M * reading + RAINFALL_CAL_B;

  // Inches per interval
  float time_since_flush = millis() - solenoid_timer;
  float intervals = time_since_flush / (RF_MEASURE_INTERVAL * 1000000.0);
  reading = reading / intervals;

  // Temporary purposes only!
  reading = 1.1;

  // Flush with solenoid if necessary
  if (reading > RAINFALL_FLUSH_LEVEL) {
    digitalWrite(SOLENOID_CONTROL_PIN, LOW);
    delay(RAINFALL_FLUSH_TIME);
    digitalWrite(SOLENOID_CONTROL_PIN, HIGH);
    solenoid_timer = millis();
  }

  return reading;
}

float measure_aerosol() {
  return 0.0;
}

float measure_atmosphere() {
  return 0.0;
}

void measure_other() {  // Template

}
