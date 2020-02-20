#include "HardwareSerial_NB_BC95.h"

#include <Adafruit_GFX.h>    // Core graphics library by Adafruit
#include <Arduino_ST7789.h> // Hardware-specific library for ST7789 (with or without CS pin)
#include <SPI.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <Wire.h>
boolean isShowTemp = false;
LiquidCrystal_PCF8574 lcd(0x3F); // set the LCD address to 0x27 for a 16 chars and 2 line display

int show = -1;
int error;
TaskHandle_t Task1;
TaskHandle_t Task2;

// Board Thingcontrol
#define TFT_DC    19
#define TFT_RST   5
#define TFT_MOSI  23   // for hardware SPI data pin (all of available pins)
#define TFT_SCLK  18   // for hardware SPI sclk pin (all of available pins)

//Arduino_ST7789 tft = Arduino_ST7789(TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK); //for display without CS pin
signal meta ;
String json = "";
String attr = "";

HardwareSerial hwSerial(2);
#define SERIAL1_RXPIN 18
#define SERIAL1_TXPIN 19
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms


String deviceToken = "rdSjoaopGUwTfksMXG0P";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;

HardwareSerial_NB_BC95 AISnb;
const long intervalSendAttribute = 600000;  //millisecond
const long interval = 30000;  //millisecond
const long intervalDisplay = 40000;  //millisecond
unsigned long previousMillis = 0;
unsigned long displayPreviousMillis = 0 ;
unsigned long attributePreviousMillis = 0;
float temp(NAN), hum(NAN), pres(NAN);

String imsi = "";
boolean readPMS = false;

struct pms7003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};


struct pms7003data data;

void _initLCD()
{


  Serial.println("LCD...");

  Serial.println("Dose: check for LCD");

  // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
//  Wire.begin(27, 16);
  Wire.begin();
  Wire.beginTransmission(0x3F);
  error = Wire.endTransmission();
  Serial.print("Error: ");
  Serial.println(error);



} // _initLCD()

void greeting() {
  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.begin(16, 2); // initialize the lcd
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
    lcd.print("Starting Dustboy");
    lcd.setCursor(0, 1);
    lcd.print(imsi);
    delay(5000);

  } else {
    Serial.println(": LCD not found.");
  } // if
}
void _initBME280()
{
  while (!Serial) {} // Wait

  delay(200);
//    Wire.begin(27, 16);
    Wire.begin();
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
}


void setup() {

  pinMode(4, OUTPUT); // turn on PMS7003
  digitalWrite(4, HIGH); // turn on PMS7003
  delay(1000);
  pinMode(32, OUTPUT); // on BME280
  digitalWrite(32, HIGH); // on BME280
  delay(1000);
  pinMode(33, OUTPUT); // on LCD
  digitalWrite(33, HIGH); // on LCD
  delay(1000);
  Serial.begin(115200);
  _initBME280();
  _initLCD();

  greeting();
  AISnb.debug = true;

  AISnb.setupDevice(serverPort);
  //
  String ip1 = AISnb.getDeviceIP();
  imsi = AISnb.getIMSI();
  imsi.trim();
  delay(4000);
  //
  pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();

  hwSerial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);

  printBME280Data();
   
}

void printBME280Data()
{
  _initBME280();
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);

}

void composeJson() {
  meta = AISnb.getSignal();
  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"temp\":");
  json.concat(temp);
  json.concat(",\"hum\":");
  json.concat(hum);
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"pm1\":");
  json.concat(data.pm01_env);
  json.concat(",\"pm2.5\":");
  json.concat(data.pm25_env);
  json.concat(",\"pm10\":");
  json.concat(data.pm100_env);

  json.concat(",\"pn03\":");
  json.concat(data.particles_03um);
  json.concat(",\"pn05\":");
  json.concat(data.particles_05um);
  json.concat(",\"pn10\":");
  json.concat(data.particles_10um);
  json.concat(",\"pn25\":");
  json.concat(data.particles_25um);
  json.concat(",\"pn50\":");
  json.concat(data.particles_50um);
  json.concat(",\"pn100\":");
  json.concat(data.particles_100um);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");

  if (data.pm25_env > 1300)
    ESP.restart();

}


void printPMS7003() {

  // reading data was successful!
  //  Serial.println();
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (standard)");
  //  Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  //  Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  //  Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  //  Serial.println("---------------------------------------");
  //  Serial.println("Concentration Units (environmental)");
  Serial.print("PM1.0:"); Serial.print(data.pm01_env);
  Serial.print("\tPM2.5:"); Serial.print(data.pm25_env);
  Serial.print("\tPM10:"); Serial.println(data.pm100_env);
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
  Serial.println("---------------------------------------");

}

void showPM() {
  _initLCD();
  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
    lcd.print("PM2.5: ");
    lcd.print(data.pm25_env);
    lcd.print(" ");
    lcd.print("ug/m3");
    lcd.setCursor(0, 1);
    lcd.print("PM10 : ");
    lcd.print(data.pm100_env);
    lcd.print(" ");
    lcd.print("ug/m3");
    delay(500);
    Serial.println("showPM");
  }
}

void showTemp() {
  _initLCD();
  if (error == 0) {
    Serial.println(": LCD found.");
    show = 0;
    lcd.setBacklight(255);
    lcd.home();
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print(" ");
    lcd.print("c");
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(hum);
    lcd.print(" ");
    lcd.print("%");
    delay(500);
    Serial.println("showTemp");
  }
}

boolean readPMSdata(Stream *s) {
  //  Serial.println("readPMSdata");
  if (! s->available()) {
    Serial.println("readPMSdata.false");
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&data, (void *)buffer_u16, 30);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void sendAttribute() {
  attr = "";
  attr.concat("{\"Tn\":\"");
  attr.concat(deviceToken);
  attr.concat("\",\"IMSI\":");
  attr.concat("\"");
  attr.concat(imsi);
  attr.concat("\"}");
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, attr);

}
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    readPMS = readPMSdata(&hwSerial);
    printBME280Data();
    printPMS7003() ;
    showPM();

    composeJson();
    Serial.println(json);
    if (readPMS) {
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
    }
    UDPReceive resp = AISnb.waitResponse();
  }

  if (currentMillis - displayPreviousMillis >= intervalDisplay) {
    displayPreviousMillis = currentMillis;
    showTemp();
  }

  if (currentMillis - attributePreviousMillis >= intervalSendAttribute) {
    attributePreviousMillis = currentMillis;
    sendAttribute();
  }

 
}
