//Final_ARDUINO
// #include <EEPROM.h>
#include "GravityTDS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

int relayPin1 = 8;
int relayPin2 = 9;  // define output pin for relay

// #define TdsSensorPin A1
// #define ONE_WIRE_BUS 7

namespace pin {
const byte tds_sensor = A1;
const byte one_wire_bus = 7;  // Dallas Temperature Sensor
}
namespace device {
float aref = 4.3;
}
SoftwareSerial Arduino_SoftwareSerial(10, 11);
// GravityTDS gravityTds;
// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x3F, 20, 4);
static float calibration_value = 21.34 - 0.46 + 0.50;
int phval = 0;
unsigned long int avgval;
int buffer_arr[10], temp;
// float temperature = 25, tdsValue = 29 , p;
// int tds = 0;
float p;
OneWire oneWire(pin::one_wire_bus);
DallasTemperature dallasTemperature(&oneWire);

float finalph, finaltemp;
int finaltds;

char c;
String datain;
int8_t indexOfA, indexOfB;

String data1, data2;
namespace sensor {
float ec = 0;
unsigned int tds = 0;
float waterTemp = 0;
float ecCalibration = 1;
}
void setup() {

  pinMode(relayPin1, OUTPUT);  // define pin 8 as output
  pinMode(relayPin2, OUTPUT);
  // sensors.begin();            // Start up the library
  Serial.begin(115200);
  dallasTemperature.begin();

  // gravityTds.setPin(TdsSensorPin);
  // gravityTds.setAref(5.0);       //reference voltage on ADC, default 5.0V on Arduino UNO
  // gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  // gravityTds.begin();            //initialization
  Serial.println("Dallas Temperature IC Control Library Demo");
  // Start up the library
  // sensors.begin();

  Arduino_SoftwareSerial.begin(9600);

  lcd.init();
  lcd.clear();
  lcd.backlight();
}
void loop() {
  {
    while (Arduino_SoftwareSerial.available() > 0) {
      c = Arduino_SoftwareSerial.read();

      if (c == '\n') {
        break;
      } else {
        datain += c;
      }
    }

    if (c == '\n') {
      ParseTheData();

      //Show all the data
      Serial.println("------------- Data coming from esp8266 -------------");
      Serial.println("Data1=  pH: " + data1);
      Serial.println("Data2= TDS: " + data2);
      Serial.println("----------------------------------------------------");

      c = 0;
      datain = "";
    }
  }

  //pH
  {
    digitalWrite(relayPin1, HIGH);
    digitalWrite(relayPin2, HIGH);
    delay(10000);
    {
      for (int i = 0; i < 10; i++) {
        buffer_arr[i] = analogRead(A0);
        delay(300);
      }
      delay(3000);
      for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
          if (buffer_arr[i] > buffer_arr[j]) {
            temp = buffer_arr[i];
            buffer_arr[i] = buffer_arr[j];
            buffer_arr[j] = temp;
          }
        }
      }
      avgval = 0;
      for (int i = 2; i < 8; i++)
        avgval += buffer_arr[i];
      float volt = (float)avgval * 5.0 / 1024 / 6;
      float ph_act = -5.70 * volt + calibration_value;

      delay(1000);
      Serial.print("pH Val:");
      Serial.println(ph_act);
      Serial.print("voltage:");
      Serial.println(volt);
      p = ph_act;
    }
  }
  // digitalWrite(relayPin1,HIGH);
  // digitalWrite(relayPin2, HIGH);
  delay(5000);
  //  Serial.print(" Requesting temperatures...");
  // sensors.requestTemperatures();  // Send the command to get temperature readings
  // sensors.requestTemperatures();
  // float t = sensors.getTempCByIndex(0);
  // Serial.println("Temp: " + (String)t);
  {
    digitalWrite(relayPin1, LOW);
    digitalWrite(relayPin2, LOW);
    // delay(10000);
    // gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    // gravityTds.update();                     //sample and calculate
    // tdsValue = gravityTds.getTdsValue();     // then get the value
    // Serial.print(tdsValue, 0);
    // Serial.println("ppm");
    // tds = tdsValue;
    readTdsQuick();
  }
  delay(5000);
  lcd.setCursor(0, 0);  //Set cursor to character 2 on line 0
  lcd.print("Temp is:" + (String)sensor::waterTemp);

  lcd.setCursor(0, 1);  //Set cursor to character 2 on line 0
  lcd.print("TDS is:" + (String)sensor::tds + "ppm");

  lcd.setCursor(0, 2);  //Set cursor to character 2 on line 0
  lcd.print("Ph is:" + (String)p + " ");

  //  delay(500);

  finalph = p;
  finaltds = sensor::tds;
  finaltemp = sensor::waterTemp;

  //  Serial.println("Data is sent from arduino");
  Arduino_SoftwareSerial.print(finalph);
  Arduino_SoftwareSerial.print("A");
  Arduino_SoftwareSerial.print(finaltds);
  Arduino_SoftwareSerial.print("B");
  Arduino_SoftwareSerial.print(finaltemp);
  Arduino_SoftwareSerial.print("C");
  Arduino_SoftwareSerial.print("\n");

  delay(500);
}

void ParseTheData() {
  indexOfA = datain.indexOf("A");
  indexOfB = datain.indexOf("B");

  data1 = datain.substring(0, indexOfA);
  data2 = datain.substring(indexOfA + 1, indexOfB);
}
void readTdsQuick() {
  dallasTemperature.requestTemperatures();
  sensor::waterTemp = dallasTemperature.getTempCByIndex(0);
  float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0;                                           // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float temperatureCoefficient = 1.0 + 0.02 * (sensor::waterTemp - 25.0);                                      // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration;                                       // temperature and calibration compensation
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5;  //convert voltage value to tds value
  Serial.print(F("TDS:"));
  Serial.println(sensor::tds);
  Serial.print(F("EC:"));
  Serial.println(sensor::ec, 2);
  Serial.print(F("Temperature:"));
  Serial.println(sensor::waterTemp, 2);
}
