// SPDX-FileCopyrightText: 2022 Limor Fried for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// CHANGE LOG
// -----------
// 2023-02-15   njeffrey  Start with factory default script for Adafruit Feather ESP32-S3 TFT https://github.com/adafruit/Adafruit_Learning_System_Guides/blob/main/Factory_Tests/Feather_ESP32S3_TFT_FactoryTest/Feather_ESP32S3_TFT_FactoryTest.ino
// 2023-02-17   njeffrey  Add Adafruit BME680 temperature / humidity / pressure sensor
// 2023-02-19   njeffrey  Add Adafruit VL53L1X time of flight sensor
// 2023-02-20   njeffrey  Add subroutine for rebooting every 24 hours
// 2023-02-21   njeffrey  Add WatchDogTimer (WDT) to reset if a sensor hangs
// 2023-02-25   njeffrey  Add webserver functionality to make sensor readings available over HTTP
// 2023-02-27   njeffrey  Lots of sensor readings, too much for single page of text on 240x135 TFT display.  Rotate info on TFT display every 5 seconds.




#include <Arduino.h>
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSans12pt7b.h>

#include <esp_task_wdt.h>
//60 seconds WDT (Watch Dog Timer) 
#define WDT_TIMEOUT 60



Adafruit_BME280 bme;  // I2C
bool bmefound = false;

extern Adafruit_TestBed TB;

Adafruit_LC709203F lc;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

GFXcanvas16 canvas(240, 135);



#include "WiFi.h"
// Enter your WiFi SSID and password
char ssid[] = "YourSSID";  // your network SSID (name)
char pass[] = "YourWiFiPassword";  // your network password (WPA,WPA2, not WEP)
int status = WL_IDLE_STATUS;
// Initialize the wifi client library
WiFiClient client;
// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;


// Adafruit VL53L1X time of flight sensor
#include "Adafruit_VL53L1X.h"
#define IRQ_PIN 2
#define XSHUT_PIN 3
Adafruit_VL53L1X vl53l1x = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Adafruit VL6180X time of flight and lux sensor
#include <Wire.h>
#include "Adafruit_VL6180X.h"
Adafruit_VL6180X vl6180x = Adafruit_VL6180X();

// Adafruit BME680 temperature, humidity, and barometic pressure sensor
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme680;  // I2C
//Adafruit_BME680 bme680(BME_CS); // hardware SPI
//Adafruit_BME680 bme680(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);



// BUG FIX: inconsistent use of variable types float / int / uint8_t / unit16_t / long / etc,etc.  Figure out optimal variable types and make more consistent.
// declare global variables for use in multiple subroutines
float   aht20_temperature_C    = 0.0; //temperature sensor from AHT20
float   aht20_humidity_pct     = 0.0; //humidity sensor from AHT20
float   bme680_temperature_C;         //sensor from BME680
float   bme680_humidity_pct;          //sensor from BME680
float   bme680_pressure_hpa;          //sensor from BME680
float   bme680_gas_resistance_kohms;  //sensor from BME680
float   bme680_altitude_m;            //sensor from BME680
float   vl53l0x_lux         = 0.0;    //light sensor from VL53L0X
uint8_t vl53l0x_range       = 0.0;    //time of flight sensor from VL53L0X
int16_t vl53l1x_range       = 0.0;    //time of flight sensor from VL53L1X
//float   vl6180x_lux         = 0.0;    //light sensor from VL6180X
//uint8_t vl6180x_range       = 0.0;    //time of flight sensor from VL6180X
int     vl6180x_lux         = 0.0;    //light sensor from VL6180X
int     vl6180x_range       = 0.0;    //time of flight sensor from VL6180X
long    wifi_rssi           = 0.0;    //signal strength for wifi connection
int     wifi_channel        = 0;      //wifi channel of current wifi connection
int     wifi_networks_found = 0;      //number of visible nearby wifi networks 
uint8_t neopixel_color      = 0;      //initialize counter variable for changing the color of the NeoPixel LED
int     battery_volt        = 0.0;    //initialize variable for battery voltage
int     battery_pct         = 0.0;    //initialize variable for battery percentage charged



// global variables available anywhere in the program, used for task scheduling and calculating uptime
unsigned long millisSinceBoot;                 //milliseconds since boot
int minutesSinceBoot;                          //minutes since boot, for scheduling tasks every X minutes
int uptime_days;                               //global variables available anywhere in the program
int uptime_hours;                              //global variables available anywhere in the program
int uptime_minutes;                            //global variables available anywhere in the program
int uptime_seconds;                            //global variables available anywhere in the program
unsigned long startMillis_TFT = millis();      //counter variable for milliseconds for refreshing TFT display screen
unsigned long startMillis_WDT = millis();      //counter variable for milliseconds for refreshing WDT (Watch Dog Timer)
unsigned long currentMillis = millis();        //counter variable for milliseconds
unsigned long startMillis_sensors = millis();  //counter variable for milliseconds for reading sensors
int WatchDogTimer = millis();                  //counter variable for hardware WatchDog
int currentPage = 0;                           //initialize variable for tracking current page being display on TFT screen
String I2C_addresses;                           //string variable to hold contents of TB.printI2CBusScan()

// variables used to check for existence of certain sensors
bool bme680_exists  = false;        //true|false flag to determine if sensor is detected at boot time
bool vl53l0x_exists = false;        //true|false flag to determine if sensor is detected at boot time
bool vl53l1x_exists = false;        //true|false flag to determine if sensor is detected at boot time
bool vl6180x_exists = false;        //true|false flag to determine if sensor is detected at boot time



void setup() {

  delay(3000);   //wait 3000ms for everything to power up
  start_WatchDogTimer();          //start WatchDog timer to reboot if the WatchDog is not fed before the timeout

  Serial.begin(115200);
  // while (! Serial) delay(10);

  


  if (!lc.begin()) {
    Serial.println(F("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!"));
    while (1) delay(10);  
  }

  Serial.println("Found LC709203F");
  Serial.print("Version: 0x");
  Serial.println(lc.getICversion(), HEX);
  lc.setPackSize(LC709203F_APA_500MAH);

  //if (TB.scanI2CBus(0x77)) {
  //  Serial.println("BME280 address");
    //unsigned status = bme.begin();
    //if (!status) {
    //  Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    //  Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    //  Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    //  Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    //  Serial.print("        ID of 0x60 represents a BME 280.\n");
    //  Serial.print("        ID of 0x61 represents a BME 680.\n");
    //  return;
    //}
    //Serial.println("BME280 found OK");
    //bmefound = true;
  //}

  
  initialize_NeoPixel_LED();      //initialize the onboard NeoPixel LED
  initialize_TFT();               //initialize the 240x135 TFT display screen
  initialize_BME680_sensor();     //initialize Adafruit BME680 temperature and barometric pressure sensor
  //initialize_VL6180X_sensor();  //initialize Adafruit VL6180X time of flight and lux sensor
  initialize_VL53L1X_sensor();    //initialize Adafruit VL53L1X time of flight and lux sensor
  initialize_wifi();              //connect to wifi network
  initialize_webserver();         //start webserver (part of Wifi.h)
  feed_WatchDog();                //feed WatchDog to reset timer to prevent reboot
}









void loop() {
  
  //start WatchDog timer to reboot if the WatchDog is not fed before the timeout
  start_WatchDogTimer();   

  //reboot the device every 24 hours to ensure no sensors have gotten locked up
  reboot_daily();

  //tell the webserver to listen for connections
  run_webserver();

  //read sensors once every 30 seconds
  if ( (millis() - startMillis_sensors) >= 30000 ) {
    read_uptime_counter();                //check uptime since last boot
    read_battery_status();                //read status of LiPo battery attached to board
    read_BME680_sensor();                 //call subroutine to read Adafruit BME680 temperature, humidity, barometric pressure sensor
    //read_VL6180X_sensor();              //call subroutine to read Adafruit VL6180X time of flight and lux sensor
    read_VL53L1X_sensor();                //call subroutine to read Adafruit VL53L1X time of flight and lux sensor
    printWifiStatus();                    //print status of wifi connection
    scan_visible_wifi_networks();         //detect wifi networks in vicinity
    read_I2C_addresses();                 //call subroutine to scan the I2C bus and return any device addresses found
    startMillis_sensors = millis();       //reset counter
  }


  // BUG FIX: when the page counter wraps from page 6 back to page 1, the incrementer mis-labels page 1 as page 7.  This is a cosmetic-only issue.
  // rotate through different text content on the TFT display screen every 5 seconds
  // use the currentPage counter to only execute the different print subroutines once per interval
  currentMillis = millis();
  if        ( (currentMillis - startMillis_TFT >=  5000) && (currentMillis - startMillis_TFT < 10000) ) {
    if (currentPage != 1) { print_to_TFT_display_page1(); currentPage = 1; }
  } else if ( (currentMillis - startMillis_TFT >= 10000) && (currentMillis - startMillis_TFT < 15000) ) {   
    if (currentPage != 2) { print_to_TFT_display_page2(); currentPage = 2; }
  } else if ( (currentMillis - startMillis_TFT >= 15000) && (currentMillis - startMillis_TFT < 20000) ) {   
    if (currentPage != 3) { print_to_TFT_display_page3(); currentPage = 3; }
  } else if ( (currentMillis - startMillis_TFT >= 20000) && (currentMillis - startMillis_TFT < 25000) ) {   
    if (currentPage != 4) { print_to_TFT_display_page4(); currentPage = 4; }
  } else if ( (currentMillis - startMillis_TFT >= 25000) && (currentMillis - startMillis_TFT < 30000) ) {   
    if (currentPage != 5) { print_to_TFT_display_page5(); currentPage = 5; }
  } else if ( (currentMillis - startMillis_TFT >= 30000) && (currentMillis - startMillis_TFT < 35000) ) {   
    if (currentPage != 6) { print_to_TFT_display_page6(); currentPage = 6; }
  } else {  
    startMillis_TFT = millis();       //reset counter
  }
  
  feed_WatchDog();   //feed WatchDog to reset timer to prevent reboot
}








void reboot_daily() {                 
  //
  // reboot every 24 hours of uptime to ensure no sensor weirdness
  //
  millisSinceBoot = millis();                      //milliseconds since boot
  if ( millisSinceBoot >= (1000 * 60 * 60 * 24) ) {
    Serial.println("Rebooting due to 24 hours uptime");
    Serial.println("**********************");
    ESP.restart();                               //ESP.reset() is a hard reset that may not clear counters, ESP.restart() is a graceful restart
  }  
}




void start_WatchDogTimer() {                 
  esp_task_wdt_init(WDT_TIMEOUT, true);           //enable panic so ESP32 restarts if the watchdog is not fed in 60 seconds
  esp_task_wdt_add(NULL);                         //add current thread to WDT watch
  if ( (millis() - startMillis_WDT) >= 60000 ) {  //to avoid spamming the serial debug port hundreds of times per second, only print every 60 seconds
    Serial.println("Setting 60 second WatchDog Timer");
    Serial.println("**********************");
    startMillis_WDT = millis();                  //reset counter
  }
}




void feed_WatchDog() {                 
  esp_task_wdt_reset();   //feed the watchdog to prevent reboot
  //Serial.println("Feeding the WatchDog to reset the WatchDog Timer");  //no need to spam the serial debug port hundreds of times per second
  //Serial.println("**********************");
}




void initialize_NeoPixel_LED() {
  Serial.println("Initalizing onboard NeoPixel LED");
  Serial.println("**********************");
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);   //turn on NeoPixel LED
  delay(10);
  TB.neopixelPin = PIN_NEOPIXEL;
  TB.neopixelNum = 1;
  TB.begin();
  TB.setColor(GREEN);
}

void set_NeoPixel_LED_red() {
  //rotate through color wheel 
  Serial.println("Changing color of NeoPixel LED to red");
  Serial.println("**********************");
  TB.setColor(RED);        //change color of NeoPixel LED 
  delay(10);
}

void set_NeoPixel_LED_green() {
  //rotate through color wheel 
  Serial.println("Changing color of NeoPixel LED to green");
  Serial.println("**********************");
  TB.setColor(GREEN);        //change color of NeoPixel LED 
  delay(10);
}

void set_NeoPixel_LED_blue() {
  //rotate through color wheel 
  Serial.println("Changing color of NeoPixel LED to blue");
  Serial.println("**********************");
  TB.setColor(BLUE);        //change color of NeoPixel LED 
  delay(10);
}

void set_NeoPixel_LED_white() {
  //rotate through color wheel 
  Serial.println("Changing color of NeoPixel LED to white");
  Serial.println("**********************");
  TB.setColor(WHITE);        //change color of NeoPixel LED 
  delay(10);
}


void initialize_TFT() {   //initialize the 240x135 TFT display screen
  //
  // turn on the TFT / I2C power supply
  //
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  display.init(135, 240);  // Init ST7789 240x135
  display.setRotation(3);
  canvas.setFont(&FreeSans12pt7b);
  canvas.setTextColor(ST77XX_WHITE);
  
  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor
  canvas.setTextColor(ST77XX_WHITE);
  canvas.println("Initializing.... ");
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  Serial.println("Initializing 240x135 TFT display screen");
  Serial.println("**********************");
}


void initialize_BME680_sensor() {
  Serial.println(F("Adafruit BME680 sensor test"));
  if (!bme680.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    Serial.println("**********************");
    bme680_exists = false;
  } else {
    bme680_exists = true;
    // Set up oversampling and filter initialization
    Serial.println("Found BME680 sensor");
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150);  // 320*C for 150 ms
  }
}





void read_BME680_sensor() {
  //
  // read from BME680 sensor
  if (!bme680.performReading()) {
    Serial.println("Failed to perform BME680 reading ");
    Serial.println("**********************");
    return;
  }
  // initialize variables with dummy values
  bme680_temperature_C        = 9999;
  bme680_humidity_pct         = 9999;
  bme680_pressure_hpa         = 9999;
  bme680_gas_resistance_kohms = 9999;
  bme680_altitude_m           = 9999;

  // save values in global variables to make available to webserver
  bme680_temperature_C        = bme680.temperature;                         //sensor from BME680
  bme680_humidity_pct         = bme680.humidity;                            //sensor from BME680
  bme680_pressure_hpa         = bme680.pressure / 100.0;                    //sensor from BME680
  bme680_gas_resistance_kohms = bme680.gas_resistance / 1000.0;             //sensor from BME680
  bme680_altitude_m           = bme680.readAltitude(SEALEVELPRESSURE_HPA);  //sensor from BME680

  // confirm sensor readings were received
  if ( (bme680_temperature_C == 9999) || (bme680_humidity_pct == 9999) || (bme680_pressure_hpa == 9999) || (bme680_gas_resistance_kohms == 9999) || (bme680_altitude_m == 9999) ) {
    Serial.println("Did not get valid readings from BME680 sensor, trying again...");
    delay(1000);
    // try to read the sensor again
    bme680_temperature_C        = bme680.temperature;                         //sensor from BME680
    bme680_humidity_pct         = bme680.humidity;                            //sensor from BME680
    bme680_pressure_hpa         = bme680.pressure / 100.0;                    //sensor from BME680
    bme680_gas_resistance_kohms = bme680.gas_resistance / 1000.0;             //sensor from BME680
    bme680_altitude_m           = bme680.readAltitude(SEALEVELPRESSURE_HPA);  //sensor from BME680
    delay(1000);
  }

  if ( (bme680_temperature_C == 9999) || (bme680_humidity_pct == 9999) || (bme680_pressure_hpa == 9999) || (bme680_gas_resistance_kohms == 9999) || (bme680_altitude_m == 9999) ) {
    Serial.println("Did not get valid readings from BME680 sensor, giving up...");
  }
  
  Serial.println("Reading BME680 sensor");
  Serial.print("Temperature = ");
  Serial.print(bme680_temperature_C);
  Serial.println(" C");

  Serial.print("Pressure = ");
  Serial.print(bme680_pressure_hpa);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme680_humidity_pct);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme680_gas_resistance_kohms);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme680_altitude_m);
  Serial.println(" m");

  Serial.println("**********************");
}




// this subroutine scans the I2C bus and returns the hexadecimal addresses of any discovered devices
// this subroutine takes ~3 seconds to run
void read_I2C_addresses() {
  I2C_addresses = " ";     //initialize variable
  Serial.println("Starting scan of I2C bus");
  TB.printI2CBusScan();   //this takes ~4 seconds, so save the results to use later
  Serial.println("Finished scan of I2C bus");
  for (uint8_t a = 0x01; a <= 0x7F; a++) {   //scan all addresses on the I2C bus, this takes ~2-3 seconds per device on the I2C bus
    if (TB.scanI2CBus(a, 0)) {
      Serial.print("0x");
      Serial.print(a, HEX);
      Serial.print(", ");
    }
  }
  Serial.println("");
  Serial.println("**********************");

}


void initialize_VL53L1X_sensor() {
  // initialize VL53L1X time of flight sensor
  Serial.println(F("Adafruit VL53L1X sensor test"));

  Wire.begin();
  if (! vl53l1x.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL53L1X sensor: "));
    Serial.println(vl53l1x.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("VL53L1X Sensor ID: 0x"));
  Serial.println(vl53l1x.sensorID(), HEX);

  if (! vl53l1x.startRanging()) {
    Serial.print(F("VL53L1X Couldn't start ranging: "));
    Serial.println(vl53l1x.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53l1x.setTimingBudget(50);
  Serial.print(F("VL53L1X Timing budget (ms): "));
  Serial.println(vl53l1x.getTimingBudget());
  Serial.println("**********************");

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
}




void read_VL53L1X_sensor() {
  // read from VL53L1X time of flight sensor
  Serial.println("Reading VL53L1X time of flight sensor");  

  // initialize variables with dummy values
  vl53l1x_range        = 9999;
  
  
  if (vl53l1x.dataReady()) {  //confirm the sensor is in a ready state
    vl53l1x_exists = true;
    
    // new measurement for the taking
    vl53l1x_range = vl53l1x.distance();
    if (vl53l1x_range == -1) {
      // something went wrong!
      Serial.print(F("VL53L1X Couldn't get distance: "));
      Serial.println(vl53l1x.vl_status);
      return;
    }
    Serial.print(F("Distance: "));
    Serial.print(vl53l1x_range);
    Serial.println(" mm");

    // data is read out, time for another reading!
    vl53l1x.clearInterrupt();
  } else {
    Serial.println("Could not read from VL53L1X time of flight sensor.  Trying again...");
    Serial.print(F("Sensor status: "));
    Serial.println(vl53l1x.vl_status);

    Serial.print(vl53l1x_range);
    vl53l1x_range = vl53l1x.distance();
    Serial.print(F("Distance: "));
    Serial.print(vl53l1x_range);
    Serial.println(" mm");
    // data is read out, time for another reading!
    vl53l1x.clearInterrupt();

  }
  Serial.println("**********************");
}


void initialize_VL6180X_sensor() {
  // initialize VL6180X time of flight sensor
  if (!vl6180x.begin()) {
    Serial.println("Failed to find VL6180X sensor");
    vl6180x_exists = false;
  } else {
    Serial.println("VL6180X time of flight Sensor found!");
    vl6180x_exists = true;
  }
}


void read_VL6180X_sensor() {
  // read from VL6180X time of flight sensor

  // initialize variables with dummy values
  vl6180x_range      = 9999;
  vl6180x_lux        = 9999;


  if (vl6180x_exists == false) {
    Serial.println("VL6180X sensor not found, skipping sensor reading");
    Serial.println("**********************");
  }
  if (vl6180x_exists == true) {
    Serial.println("Reading VL6180X sensor");

    vl6180x_lux = vl6180x.readLux(VL6180X_ALS_GAIN_5);

    Serial.print("Lux: ");
    Serial.print(vl6180x_lux);
    Serial.println(" lumens");

    vl6180x_range = vl6180x.readRange();
    uint8_t status = vl6180x.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
      Serial.print("Range: ");
      Serial.print(vl6180x_range);
      Serial.println(" mm");
    }

    // Some error occurred, print it out!

    if ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
      Serial.println("Range: System error");
    } else if (status == VL6180X_ERROR_ECEFAIL) {
      Serial.println("Range: ECE failure");
    } else if (status == VL6180X_ERROR_NOCONVERGE) {
      Serial.println("Range: No convergence");
    } else if (status == VL6180X_ERROR_RANGEIGNORE) {
      Serial.println("Range: Ignoring range");
    } else if (status == VL6180X_ERROR_SNR) {
      Serial.println("Range: Signal/Noise error");
    } else if (status == VL6180X_ERROR_RAWUFLOW) {
      Serial.println("Range: Raw reading underflow");
    } else if (status == VL6180X_ERROR_RAWOFLOW) {
      Serial.println("Range: Raw reading overflow");
    } else if (status == VL6180X_ERROR_RANGEUFLOW) {
      Serial.println("Range: reading underflow");
    } else if (status == VL6180X_ERROR_RANGEOFLOW) {
      Serial.println("Range: reading overflow");
    }
    Serial.println("**********************");
  }
}


void initialize_wifi() {
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);

  WiFi.disconnect();
  WiFi.begin(ssid, pass);
  delay(1000);              //wait for 1000ms to connect to WiFi

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed, trying again...");
    WiFi.disconnect();           //disconnection should not be needed, but just in case
    WiFi.begin(ssid, pass);      //attempt to connect to WiFi
    delay(3000);                 //wait for 3000ms for WiFi to connect

  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed, trying again...");
    WiFi.disconnect();           //disconnection should not be needed, but just in case
    WiFi.begin(ssid, pass);      //attempt to connect to WiFi
    delay(3000);                 //wait for 3000ms for WiFi to connect
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed, trying again...");
    WiFi.disconnect();           //disconnection should not be needed, but just in case
    WiFi.begin(ssid, pass);      //attempt to connect to WiFi
    delay(3000);                 //wait for 3000ms for WiFi to connect
  }
  if (WiFi.status() != WL_CONNECTED) {                           //give up after 3 attempts
    Serial.print("ERROR: Unable to connect to WiFi network with SSID ");
    Serial.println(ssid);
  }

  if (WiFi.status() != WL_CONNECTED) {                           //reboot if no WiFi connection after 5 minutes uptime
    if ( millisSinceBoot > (1000*60*5) ) {
      Serial.println("No WiFi connection for 5 minutes, rebooting!");
      ESP.restart();                               //ESP.reset() is a hard reset that may not clear counters, ESP.restart() is a graceful restart
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to WiFi network ");
    Serial.println(WiFi.SSID());
  }
  Serial.println("**********************");
}




void printWifiStatus() {

  //confirm WiFi is currently connected
  // more robust example at https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/#4
  // example at https://randomnerdtutorials.com/solved-reconnect-esp32-to-wifi/
  //reconnect to wifi if connection is lost
  if (WiFi.status() != WL_CONNECTED) {
    initialize_wifi();
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  wifi_rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(wifi_rssi);
  Serial.println(" dBm");

  // print the WiFi channel
  wifi_channel = WiFi.channel();
  Serial.print("Channel:");
  Serial.println(wifi_channel);
  
  // print your board's MAC address:
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // print your board's IP address, subnet mask, gateway
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());

  Serial.println("**********************");
}





// BUG FIX: this will run multiple times when minutesSinceBoot % 5 = 0, add some logic to make it only run once, similar to TFT screen refreshes
void scan_visible_wifi_networks() {
  //look at nearby wifi networks, useful for troubleshooting interference
  // WiFi.scanNetworks will return the number of networks found
  
  // Performing a scan for nearby WiFi networks causes a very brief wifi disconnection (usually just a single ping packet is lost), 
  // so keep cached data around for 5 minutes.  In other words, only run the WiFi.scanNetworks() command every 5 minutes to minimize network connectivity drops

  millisSinceBoot = millis();                      //milliseconds since boot
  minutesSinceBoot = millisSinceBoot / 1000 / 60;  //convert milliseconds to minutes 
  if ( (minutesSinceBoot > 1) && (minutesSinceBoot % 5 != 0) ) { 
    Serial.println("Skipping scan of nearby WiFi networks, using cached data for up to 5 minutes");    
    for (int i = 0; i < wifi_networks_found; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (RSSI ");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm, Channel ");
      Serial.print(WiFi.channel(i));
      Serial.println(")");
    }
  } else {
    // we only get this far if the number of minutes of uptime is evenly divisible by 5, so this section only runs every 5 minutes
    Serial.println("Scanning for nearby WiFi networks");

     if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();           //disconnection should not be needed, but just in case
      delay(3000);                 //wait for the WiFi disconnection to complete
    }

    wifi_networks_found = 0;                                          //initialize variable to ensure we are not using cached info from last run
    wifi_networks_found = WiFi.scanNetworks();                        //this is a blocking operation that takes a few hundred milliseconds, and briefly disconnects any existing WiFi connections, ususally causing 1 ping timeout
    //delay(1000);                                                      //wait for the scan to complete

    if (wifi_networks_found == -2) {                                  //WIFI_SCAN_FAILED -2 , might be because you did not wait long enough, try increasing delay(###)
      Serial.println("Error: WiFi.scanNetworks() WIFI_SCAN_FAILED ");
    } else if (wifi_networks_found == -1) {                           //WIFI_SCAN_RUNNING -1
      Serial.println("Error: WiFi.scanNetworks() WIFI_SCAN_RUNNING failed!");
    } else if (wifi_networks_found == 0) {                            //scan completed successfully, but did not see any nearby WiFi networks
      Serial.println("no WiFi networks found");
    } else if (wifi_networks_found > 0) {                             //scan completed successfully, saw at least 1 nearby WiFi network
      Serial.print(wifi_networks_found);
      Serial.println(" WiFi networks found");
      for (int i = 0; i < wifi_networks_found; i++) {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);      //add 1 because counter starts at 0
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (RSSI ");
        Serial.print(WiFi.RSSI(i));
        Serial.print(" dBm, Channel ");
        //printEncryptionType(WiFi.encryptionType(i));
        //if      (WiFi.encryptionType(i) == ENC_TYPE_WEP)  { Serial.print("WEP, Channel ");}
        //else if (WiFi.encryptionType(i) == ENC_TYPE_TKIP) { Serial.print("WPA, Channel ");}
        //else if (WiFi.encryptionType(i) == ENC_TYPE_CCMP) { Serial.print("WPA2, Channel ");}
        //else if (WiFi.encryptionType(i) == ENC_TYPE_NONE) { Serial.print("None, Channel ");}
        //else if (WiFi.encryptionType(i) == ENC_TYPE_OPEN) { Serial.print("Open, Channel ");}
        //else if (WiFi.encryptionType(i) == ENC_TYPE_AUTO) { Serial.print("Auto, Channel ");}
        //else { Serial.print("Unknown, Channel ");}
        Serial.print(WiFi.channel(i));
        //Serial.print(", BSSID ");
        //Serial.print(WiFi.BSSID(i));
        Serial.println(")");
        delay(10);
      }
    }
  }
  Serial.println("**********************");
}




void initialize_webserver() {
  server.begin();
}




void run_webserver() {
  WiFiClient client = server.available();  // Listen for incoming clients

  if (client) {  // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New HTTP Client.");                                        // print a message out in the serial port
    String currentLine = "";                                                   // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {  // if there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();


            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head>"); 
            client.println("   <meta http-equiv=refresh content=60>");  //tell web browser to refresh every 60 seconds
            client.println("   <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("   <link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("   <style>");
            client.println("      html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println("      .button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("      text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println("      .button2 {background-color: #555555;}");
            client.println("   </style>");
            client.println("</head>");
            client.println("<body>");


            // Web Page Heading
            client.println("<h1>ESP32 Web Server</h1>");

            //show uptime
            client.print("<p>Uptime: ");
            client.print(uptime_days);
            client.print(" days, ");
            client.print(uptime_hours);
            client.print(" hours, ");
            client.print(uptime_minutes);
            client.print(" minutes, ");
            client.print(uptime_seconds);
            client.println(" seconds. </p>");
 
            //LiPo battery status
            client.print("<p>Battery: ");
            //client.print(lc.cellVoltage(), 1);
            client.print(battery_volt, 1);
            client.print(" V / ");
            //client.print(lc.cellPercent(), 0);
            client.print(battery_pct, 0);
            client.println("% </p>");

            // Display current sensor readings
            ////AHT20 temperature and humidity sensor
            //client.print("<p>Temperature: ");
            //client.print(aht20_temperature_C);
            //client.println(" C </p>");
            ////AHT20 temperature and humidity sensor
            //client.print("<p>Humidity: ");
            //client.print(aht20_humidity_pct);
            //client.println(" % rH </p>");

            //BME680 temperature, humidity, barometric pressure, gas sensor
            client.print("<p>Temperature: ");
            client.print(bme680_temperature_C);
            client.println(" C </p>");
            client.print("<p>Humidity: ");
            client.print(bme680_humidity_pct);
            client.println(" % rH </p>");
            client.print("<p>Barometric Pressure: ");
            client.print(bme680_pressure_hpa);
            client.println(" hPa</p>");
            client.print("<p>Gas Resistance: ");
            client.print(bme680_gas_resistance_kohms);
            client.println(" KOhms</p>");
            client.print("<p>Altitude: ");
            client.print(bme680_altitude_m);
            client.println(" m</p>");

            ////VL53L0X time of flight and lux sensor
            //client.print("<p>Distance: ");
            //client.print(vl53l0x_range);
            //client.print("<p>Lux: ");
            //client.print(vl53l0x_lux);
            //client.println(" lumens </p>");

            //VL53L1X time of flight sensor
            client.print("<p>Distance: ");
            client.print(vl53l1x_range);
            client.println(" mm </p>");

            //VL6180X time of flight lux sensor
            //client.print("<p>Lux: ");
            //client.print(vl6180x_lux);
            //client.println(" lumens </p>");
            //client.print("<p>Distance: ");
            //client.print(vl6180x_range);
            //client.println(" mm </p>");

            // WiFi connection details
            client.print("<hr><p><b>WiFi Details: </b>");
            client.print("<br>MAC address: ");
            client.println(WiFi.macAddress());
            client.print("<br>IP address: ");
            client.println(WiFi.localIP());
             client.print("<br>Subnet mask: ");
            client.println(WiFi.subnetMask());
            client.print("<br>Gateway: ");
            client.println(WiFi.gatewayIP());
            client.print("<br>SSID: ");
            client.println(WiFi.SSID());
            client.print("<br>RSSI signal strength: ");
            client.print(wifi_rssi);
            client.println(" dBm ");
            client.print("<br>WiFi channel: ");
            client.println(wifi_channel);
            client.println("<hr>");

            //show nearby WiFi networks (This is useful for troubleshooting interference)
            //these values may be ~30 seconds out of date, as the details are gathered when the scan_for_visible_wifi_networks subroutine is run
            client.println("<p><b>Nearby WiFi Networks:</b>");
            for (int i = 0; i < wifi_networks_found; ++i) {
              // Print SSID and RSSI for each network found
              client.print("<br>");
              client.print(i + 1);
              client.print(": ");
              client.print(WiFi.SSID(i));
              client.print(" (RSSI ");
              client.print(WiFi.RSSI(i));
              client.print(" dBm, Channel ");
              client.print(WiFi.channel(i));
              client.println(")");
            }
            client.println("<hr>");





            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("HTTP client disconnected.");
    Serial.println("**********************");
  }
}



void read_battery_status() {
  //
  // read from LiPo battery attached to board
  // this takes ~3-4 seconds to read battery, so read sensor in this subroutine and save the values for use in other locations
  //
  battery_volt = lc.cellVoltage();   //read battery voltage
  battery_pct = lc.cellPercent();    //read battery percentage charged
  //
  // print the values
  //
  Serial.print("Battery: ");
  //Serial.print(lc.cellVoltage(), 1);
  Serial.print(battery_volt, 1);          //BUG FIX: what does the ,1 do?
  Serial.print(" V / ");
  //Serial.print(lc.cellPercent(), 0);    //BUG FIX: what does the ,0 do?
  Serial.print(battery_pct, 0);
  Serial.println("%");
  Serial.println("**********************");
}






void read_uptime_counter() {
  //
  // calculate uptime
  //
  uptime_days    = 0;                              //initalize variable
  uptime_hours   = 0;                              //initalize variable
  uptime_minutes = 0;                              //initalize variable
  uptime_seconds = 0;                              //initalize variable
  millisSinceBoot = millis();                      //milliseconds since boot
  int remainder_seconds = millisSinceBoot/1000;    //convert milliseconds to nearest second
  uptime_days = remainder_seconds / 86400;         //86400 seconds per day
  if (uptime_days > 0) { remainder_seconds = remainder_seconds - (uptime_days * 86400);}
  uptime_hours = remainder_seconds / 3600;         //3600 seconds per hour
  if (uptime_hours > 0) { remainder_seconds = remainder_seconds - (uptime_hours * 3600);}
  uptime_minutes = remainder_seconds / 60;         //60 seconds per minute
  if (uptime_minutes > 0) { remainder_seconds = remainder_seconds - (uptime_minutes * 60);}
  uptime_seconds = remainder_seconds;  
  
  Serial.print("Uptime: ");
  Serial.print(uptime_days);
  Serial.print(" days, ");
  Serial.print(uptime_hours);
  Serial.print(" hours, ");
  Serial.print(uptime_minutes);
  Serial.print(" minutes, ");
  Serial.print(uptime_seconds);
  Serial.println(" seconds. ");
  Serial.println("**********************");
}






void print_to_TFT_display_page1() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (sensor info)");
  Serial.println("**********************");

  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_RED);
  canvas.print("Temperature:");
  canvas.print(bme680_temperature_C);
  canvas.println(" C");

  canvas.setTextColor(ST77XX_YELLOW);
  canvas.print("Humidity: ");
  canvas.print(bme680_humidity_pct);
  canvas.println(" % rH");

  canvas.setTextColor(ST77XX_GREEN);
  canvas.println(" ");  //blank line
  canvas.print("Distance: ");
  canvas.print(vl53l1x_range);
  canvas.println(" mm");

  //canvas.setTextColor(ST77XX_BLUE);
  //canvas.print("Lux: ");
  //canvas.print(vl6180x_lux);
  //canvas.println(" lumens");

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}




void print_to_TFT_display_page2() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (sensor info)");
  Serial.println("**********************");

  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_BLUE);
  canvas.print("Pressure: ");
  canvas.print(bme680_pressure_hpa);
  canvas.println(" hPa");

  canvas.setTextColor(ST77XX_GREEN);
  canvas.print("Altitude: ");
  canvas.print(bme680_altitude_m);
  canvas.println(" m");

  canvas.setTextColor(ST77XX_RED);
  canvas.println("Gas Resistance: ");
  canvas.print(bme680_gas_resistance_kohms);
  canvas.println(" KOhms");

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}






void print_to_TFT_display_page3() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (WiFi network info)");
  Serial.println("**********************");

  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_WHITE);
  canvas.println("MAC address:");
  canvas.println(WiFi.macAddress());
  
  canvas.setTextColor(ST77XX_GREEN);
  canvas.print("SSID: ");
  canvas.println(WiFi.SSID());
  
  canvas.setTextColor(ST77XX_RED);
  canvas.print("RSSI: ");
  canvas.print(wifi_rssi);
  canvas.println(" dBm");
  
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}




void print_to_TFT_display_page4() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (IP address)");
  Serial.println("**********************");

  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_GREEN);
  canvas.print("IP:");
  canvas.println(WiFi.localIP());
  
  canvas.setTextColor(ST77XX_BLUE);
  canvas.print("SM: ");
  canvas.println(WiFi.subnetMask());
  
  canvas.setTextColor(ST77XX_WHITE);
  canvas.print("GW: ");
  canvas.println(WiFi.gatewayIP());
  
  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}



void print_to_TFT_display_page5() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (battery details)");
  Serial.println("**********************");
  //
  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_GREEN);
  canvas.print("Battery: ");
  //canvas.print(lc.cellVoltage(), 1);
  canvas.print(battery_volt, 1);
  canvas.print(" V  /  ");
  //canvas.print(lc.cellPercent(), 0);
  canvas.print(battery_pct, 0);
  canvas.println(" %");

  //this is a blocking operation, so save the values in the read_xxx subroutine and just print them out here
  //canvas.setTextColor(ST77XX_WHITE);
  //canvas.println(" ");  //blank line
  //canvas.println("I2C addresses: ");
  //for (uint8_t a = 0x01; a <= 0x7F; a++) {   //scan all addresses on the I2C bus
  //  if (TB.scanI2CBus(a, 0)) {
  //    canvas.print("0x");
  //    canvas.print(a, HEX);
  //    canvas.print(", ");
  //  }
  //}
  //canvas.println(" ");  //blank line
  //canvas.println("I2C addresses: "); 
  //canvas.println(I2C_addresses); 

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}




void print_to_TFT_display_page6() {
  //
  // This subroutine prints a page of information to the 240x135 TFT display screen
  //
  Serial.print("Changing TFT display from page "); Serial.print(currentPage); Serial.print(" to page "); Serial.print(currentPage+1);
  Serial.println(" (uptime counter)");
  Serial.println("**********************");
  //
  canvas.fillScreen(ST77XX_BLACK);  //overwrite entire screen with black
  canvas.setCursor(0, 25);          //place cursor

  canvas.setTextColor(ST77XX_ORANGE);
  canvas.print("Uptime: ");
  canvas.print(uptime_days);
  canvas.println(" days, ");
  canvas.print(uptime_hours);
  canvas.println(" hours, ");
  canvas.print(uptime_minutes);
  canvas.println(" minutes, ");
  canvas.print(uptime_seconds);
  canvas.println(" seconds. ");

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), 240, 135);  //redraw the TFT display
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
}



 



