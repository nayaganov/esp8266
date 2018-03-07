#include <SoftwareSerial.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPClient.h>
#include "BME280_MOD-1022.h"

const char* ssid = "OpenTechGroup";
const char* password = "korkemwifi";
WiFiClient client; // All fucntions https://www.arduino.cc/en/Reference/WiFi & http://esp8266.github.io/Arduino/versions/2.0.0/doc/libraries.html#wifi-esp8266wifi-library
//#define ledPin D9         // the onboard LED

unsigned long CO2Channel = 437367; //ThingSpeak ID
const char * WriteAPIKey = "RAN3R07A1U514BNW"; //ThingSpeak Write API

const char* host = "weatherstation.wunderground.com";
const char* WUID    = "IASTANA15"; //Weather Underground ID
const char* WUPASS  = "r3zba702"; //Weather Underground Password
//float altitudepws = 1138.00;      //LOCAL Alitude of the PWS to get relative pressure

//unsigned long MyMillis = 600000; //600000 = 10 min
unsigned long MyMillis = 60000; //in msec; 60000 = 60 sec = 1 min
//unsigned long MyMillis = 20000; //20000 = 20 sec
unsigned long MillisLast = 0;
unsigned long CurrentMillis = 0;
const unsigned long MyMillisLimit = 43200000; //restart every 12 hours

int ppm;      //CO2 ppm data
int tempCO2;  //CO2 temperature data
int cct;      //PM2.5 data
byte StatusCO2ppm = 0;
byte StatusCO2temp = 0;
byte StatusPM2_5 = 0;
byte StatusBME280 = 0;
float temp, humidity,  pressure, pressureMoreAccurate;
double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;

SoftwareSerial mySerialCO2(D5, D6); // RX,TX - CO2
byte cmdCO2[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; //data request
unsigned char responseCO2[9];

SoftwareSerial mySerialPM2_5(D7, D9); // RX,TX - PM2.5
byte progPM2_5[7] = {0x42, 0x4d, 0xe1, 0x00, 0x01, 0x01, 0x71}; //command for change from active to passive mode
byte cmdPM2_5[7] =  {0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71}; //data request
unsigned char responsePM2_5[32];
unsigned char TempResponsePM2_5[63];



void setup() {
  //pinMode(ledPin, OUTPUT);
  Serial.println("MeteoStation");
  Wire.begin();
  Serial.begin(115200);
  mySerialCO2.begin(9600);
  mySerialPM2_5.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("WiFi is ready...");
  int lo = WiFi.RSSI();
  Serial.print("WiFi signal: ");
  Serial.println(lo);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //digitalWrite(ledPin, HIGH);
  Serial.println("Let us go....");

  uint8_t chipID = BME280.readChipId();

  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
  // We'll switch into normal mode for regular automatic samples
  BME280.writeStandbyTime(tsb_0p5ms);         // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);       // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os2x);  // temperature x2
  BME280.writeOversamplingHumidity(os1x);     // humidity x1
  BME280.writeMode(smNormal);

  Serial.println(String("* * * * *"));
  //mySerialPM2_5.write(progPM2_5, 7); // change from active to passive mode
  delay(2000);
}

void loop() {
  CurrentMillis = millis();
  Serial.println("MillisLast: " + String(MillisLast));
  Serial.println("CurrentMillis: " + String(CurrentMillis));
  Serial.println("MyMillis: " + String(MyMillis));
  Serial.println("-=-");
  if (CurrentMillis > MyMillisLimit){
    Serial.println("Millis limit is reached! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  if (CurrentMillis - MillisLast > MyMillis || MillisLast == 0) {
    MillisLast = CurrentMillis;
    getCO2Data();
    getPM2_5Data();
    getTHPData();
    SendData2TS(); //ThingSpeak
    SendData2WU(); //Weather Underground
    Serial.println("* * * * *");
  }
  delay(15000);
}

/* Methods */
void getCO2Data() {
  memset(responseCO2, 0, 9);
  mySerialCO2.write(cmdCO2, 9);
  mySerialCO2.readBytes(responseCO2, 9);

  // CRC CO2 check
  int i;
  byte crcCO2 = 0;
  for (i = 1; i < 8; i++) crcCO2 += responseCO2[i];
  crcCO2 = 255 - crcCO2;
  crcCO2++;
  // End of CRC CO2 check

  if ( !(responseCO2[0] == 0xFF && responseCO2[1] == 0x86 && responseCO2[8] == crcCO2) ) {
    //Serial.println(String("-=-"));
    Serial.println("CO2 CRC error: " + String(crcCO2) + " / " + String(responseCO2[8]));
    Serial.println(String("RAW:"));
    char raw[32];
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
            responseCO2[0], responseCO2[1], responseCO2[2], responseCO2[3], responseCO2[4],
            responseCO2[5], responseCO2[6], responseCO2[7], responseCO2[8]);
    Serial.println(raw);
    MillisLast = MyMillis - 10000;
  }
  else {
    unsigned int responseCO2High = (unsigned int) responseCO2[2];
    unsigned int responseCO2Low = (unsigned int) responseCO2[3];
    ppm = (256 * responseCO2High) + responseCO2Low;
    tempCO2 = responseCO2[4] - 20;
    Serial.println(String("CO2 Data:"));
    char raw[32];
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
            responseCO2[0], responseCO2[1], responseCO2[2], responseCO2[3], responseCO2[4],
            responseCO2[5], responseCO2[6], responseCO2[7], responseCO2[8]);
    Serial.println(raw);
    if (ppm <= 250 || ppm > 4900) {
      Serial.println("CO2: no valid data");
      MillisLast = MyMillis;
    }
    else {
      StatusCO2ppm = 1;
      StatusCO2temp = 1;
    }
  }
}

void getPM2_5Data() {
  //unsigned char TempResponsePM2_5[63];
  memset(responsePM2_5, 0, 32);
  memset(TempResponsePM2_5, 0, 63);
  //mySerialPM2_5.write(cmdPM2_5, 7);
  //mySerialPM2_5.readBytes(responsePM2_5, 32);
  mySerialPM2_5.readBytes(TempResponsePM2_5, 63);
  if (TempResponsePM2_5[0] == 0x42 && TempResponsePM2_5[1] == 0x4D) {
    for (int i = 0; i < 32; i++) responsePM2_5[i] = TempResponsePM2_5[i];
  }
  else {
    for (int i = 0; i < 35; i++) {
      if (TempResponsePM2_5[i] == 0x42 && TempResponsePM2_5[i + 1] == 0x4D) {
        for (int j = 0; j < 32; j++) responsePM2_5[j] = TempResponsePM2_5[(j + i)];
        break;
      }
    }
  }

  // CRC PM2.5 check
  //int i;
  short crcPM2_5 = 0;
  for (int i = 0; i < 29; i++) crcPM2_5 += responsePM2_5[i];
  // End of CRC PM2.5 check

  if (!(responsePM2_5[0] == 0x42 && responsePM2_5[1] == 0x4d && crcPM2_5 == ((responsePM2_5[30] << 8) + responsePM2_5[31]))) {
    Serial.println("PM2.5 CRC error: " + String(crcPM2_5) + " / " + String(responsePM2_5[30]) + String(responsePM2_5[31]));
    char raw[32];
    Serial.println(String("-=-"));
    Serial.println(String("RAW:"));
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[0], responsePM2_5[1], responsePM2_5[2], responsePM2_5[3],
            responsePM2_5[4], responsePM2_5[5], responsePM2_5[6], responsePM2_5[7]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[8], responsePM2_5[9], responsePM2_5[10], responsePM2_5[11],
            responsePM2_5[12], responsePM2_5[13], responsePM2_5[14], responsePM2_5[15]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[16], responsePM2_5[17], responsePM2_5[18], responsePM2_5[19],
            responsePM2_5[20], responsePM2_5[21], responsePM2_5[22], responsePM2_5[23]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[24], responsePM2_5[25], responsePM2_5[26], responsePM2_5[27],
            responsePM2_5[28], responsePM2_5[29], responsePM2_5[30], responsePM2_5[31]);
    Serial.println(raw);
    //MillisLast = MyMillis - 10000;
  }
  else {
    cct = (responsePM2_5[6] << 8) + responsePM2_5[7];
    Serial.println(String("-=-\nPM2_5 Data:"));
    char raw[32];
    //Serial.println(String("RAW data:"));
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[0], responsePM2_5[1], responsePM2_5[2], responsePM2_5[3],
            responsePM2_5[4], responsePM2_5[5], responsePM2_5[6], responsePM2_5[7]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[8], responsePM2_5[9], responsePM2_5[10], responsePM2_5[11],
            responsePM2_5[12], responsePM2_5[13], responsePM2_5[14], responsePM2_5[15]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[16], responsePM2_5[17], responsePM2_5[18], responsePM2_5[19],
            responsePM2_5[20], responsePM2_5[21], responsePM2_5[22], responsePM2_5[23]);
    Serial.println(raw);
    sprintf(raw, "%02X %02X %02X %02X %02X %02X %02X %02X",
            responsePM2_5[24], responsePM2_5[25], responsePM2_5[26], responsePM2_5[27],
            responsePM2_5[28], responsePM2_5[29], responsePM2_5[30], responsePM2_5[31]);
    Serial.println(raw);
    if (cct < 0 || cct >= 1000) {
      Serial.println("PM2.5: no valid data");
      //MillisLast = MyMillis;
    }
    else {
      StatusPM2_5 = 1;
    }
  }
}

void printFormattedFloat(float x, uint8_t precision) {
  char buffer[10];
  dtostrf(x, 7, precision, buffer);
  Serial.print(buffer);
}

void getTHPData() {
  while (BME280.isMeasuring()) {
    //Serial.println("Measuring...");
    //delay(100);
  }
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  //float temp, humidity,  pressure, pressureMoreAccurate;
  //double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
  char buffer[80];

  temp      = BME280.getTemperature();
  humidity  = BME280.getHumidity();
  pressure  = BME280.getPressure();

  pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above

  tempMostAccurate     = BME280.getTemperatureMostAccurate();
  humidityMostAccurate = BME280.getHumidityMostAccurate();
  pressureMostAccurate = BME280.getPressureMostAccurate();

  Serial.println(String("-=-"));
  Serial.print("Temperature: ");
  //printFormattedFloat(tempMostAccurate, 2);
  printFormattedFloat(temp, 2);
  Serial.println();

  Serial.print("Humidity: ");
  //printFormattedFloat(humidityMostAccurate, 2);
  printFormattedFloat(humidity, 2);
  Serial.println();

  Serial.print("Pressure: ");
  //printFormattedFloat(pressureMostAccurate, 2);
  printFormattedFloat(pressure, 2);
  Serial.println();


  // Post data to ThingSpeak
  if (tempMostAccurate == 0 && humidityMostAccurate == 0 && pressureMostAccurate == 0) {
    Serial.print("BME: no data");
    StatusBME280 = 0;
  }
  StatusBME280 = 1;
  //postData(tempMostAccurate, humidityMostAccurate, pressureMostAccurate);
  //Serial.println();

}


void SendData2TS() {
  if (StatusCO2ppm == 1 || StatusCO2temp == 1 || StatusPM2_5 == 1 || StatusBME280 == 1) {
    Serial.println("-=-\nThingSpeak\nCO2, ppm:" + String(ppm) +
                   "\nTempCO2, C:" + String(tempCO2) +
                   "\nPM2.5, ug/m3:" + String(cct) +
                   "\nTemperature, C:" + String(tempMostAccurate) +
                   "\nHumidity, %:" + String(humidityMostAccurate) +
                   "\nPressure, hPa:" + String(pressureMostAccurate));
    String sURL = "http://api.thingspeak.com/update?api_key=";
    sURL += WriteAPIKey;
    if (StatusCO2ppm == 1) {
      sURL += "&field1=" + String(ppm);
      //StatusCO2ppm = 0;
    }
    /*    if (StatusCO2temp == 1) {
          sURL += "&field2=" + String(tempCO2);
          StatusCO2temp = 0;
        }*/
    if (StatusPM2_5 == 1) {
      sURL += "&field2=" + String(cct);
      //StatusPM2_5 = 0;
    }
    if (StatusBME280 == 1) {
      sURL += "&field3=" + String(tempMostAccurate);
      sURL += "&field4=" + String(humidityMostAccurate);
      sURL += "&field5=" + String(pressureMostAccurate);
      //StatusBME280 = 0;
    }

    Serial.print("Requesting URL: ");
    Serial.println(sURL);
    HTTPClient http;
    http.begin(sURL);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      Serial.printf("ThingSpeak responce code: %d\n", httpCode);
    }
    else {
      Serial.println("ThingSpeak failed, error: " + String(http.errorToString(httpCode).c_str()));
      MillisLast = MyMillis - 10000;
    }
  }
}


void SendData2WU() {
  if (StatusCO2ppm == 1 || StatusCO2temp == 1 || StatusPM2_5 == 1 || StatusBME280 == 1) {

    //converting data from metric to imperial system
    tempCO2 = tempCO2 * 1.8 + 32;
    tempMostAccurate = tempMostAccurate * 1.8 + 32;
    pressureMostAccurate = pressureMostAccurate / 33.864;

    Serial.println("-=-");
    Serial.println("Weather Underground\nCO2, ppm:" + String(ppm) +
                   "\nTempCO2, F:" + String(tempCO2) +
                   "\nPM2.5, ug/m3:" + String(cct) +
                   "\nTemperature, F:" + String(tempMostAccurate) +
                   "\nHumidity, %:" + String(humidityMostAccurate) +
                   "\nPressure, inHg:" + String(pressureMostAccurate));

    String sURL = "/weatherstation/updateweatherstation.php?ID=";
    sURL += WUID;
    sURL += "&PASSWORD=";
    sURL += WUPASS;
    sURL += "&dateutc=now";
    if (StatusCO2ppm == 1) {
      sURL += "&AqCO=" + String(ppm);
      StatusCO2ppm = 0;
    }
    if (StatusPM2_5 == 1) {
      sURL += "&AqPM2.5=" + String(cct);
      StatusPM2_5 = 0;
    }
    if (StatusBME280 == 1) {
      sURL += "&tempf=" + String(tempMostAccurate);
      sURL += "&humidity=" + String(humidityMostAccurate);
      sURL += "&baromin=" + String(pressureMostAccurate);
      StatusBME280 = 0;
    }
    sURL += "&action=updateraw";

    WiFiClient client;
    if (!client.connect(host, 80)) {
      Serial.println("connection failed");
      return;
    }
    
    Serial.print("Requesting URL: ");
    Serial.println(sURL);
    
    // This will send the request to the server
    client.print(String("GET ") + sURL + " HTTP/1.1\r\n" + "Host: " + host + 
    "\r\n" + "Connection: close\r\n\r\n");
    delay(10);
/*
    Serial.print (String("---\n"));
    Serial.print(String("GET ") + sURL + " HTTP/1.1\r\n" + "Host: " + host + 
    "\r\n" + "Connection: close\r\n\r\n");
    Serial.print (String("---\n"));
    // Read all the lines of the reply from server and print them to Serial
*/
    while(client.available()){
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    
    //Serial.println();
    Serial.println("closing connection");
    

  }
}

