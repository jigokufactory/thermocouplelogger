#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Ethernet.h>

#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define BME280_ADDRESS 0x76
unsigned long int hum_raw, temp_raw, pres_raw;
signed long int t_fine;
unsigned int timeout_count;
unsigned int timeout;
unsigned long int sleep_timeout;

uint32_t offset = 0;
uint32_t sleep_count;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

byte MAC[] = { 0x5e, 0xcf, 0x7f, 0xb4, 0x05, 0x09, };
byte IP[] = { 169, 254, 4, 1 };

/************************* WiFi Access Point *********************************/

//#define WLAN_SSID       "test_server"
//#define WLAN_PASS       "morisoba2"
#define WLAN_SSID       "Buffalo-G-A140"
#define WLAN_PASS       "sncxskf4bnbkx"

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO           3
#define MAXCS           4
#define MAXCLK          5

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);


// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

EthernetServer server = EthernetServer(8080);

//
void setupWiFi() {
  sleep_timeout = (4000);
  
  Serial.println(sleep_count);
  sleep_count ++;

 // if (sleep_count <= 10) {
 //   Serial.println("まだ眠い");
 //   Serial.println("ZZZ");
 //   ESP.deepSleep(sleep_timeout * 1000 * 1000 , WAKE_RF_DEFAULT);
 //   delay(750);
 // }


    
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  Serial.println(WLAN_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  timeout_count == 0;
  sleep_count == 0;
  

  
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  timeout_count ++;
  Serial.print(".");
  //↓↓約10秒以内でつながらないときはtimeoutして指定時間スリープ
    if (timeout_count >= 10) {
      Serial.println("接続がタイム・アウトしました");
      Serial.println("指定時間後に再試行...");
      Serial.println("sleep mode...");
      delay(1000);
      ESP.deepSleep(sleep_timeout * 1000 * 1000 , WAKE_RF_DEFAULT); 
      delay(1000);
      }
  }
  Serial.println();

  Serial.println("WiFi connected");
  delay(1000);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
//  uint8_t osrs_t = 1;             //Temperature oversampling x 1
//  uint8_t osrs_p = 1;             //Pressure oversampling x 1
//  uint8_t osrs_h = 1;             //Humidity oversampling x 1
//  uint8_t mode = 3;               //Normal mode
//  uint8_t t_sb = 5;               //Tstandby 1000ms
//  uint8_t filter = 0;             //Filter off
//  uint8_t spi3w_en = 0;           //3-wire SPI Disable

//  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
//  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
//  uint8_t ctrl_hum_reg  = osrs_h;

  Serial.begin(9600);
  Wire.begin();
  HTTPClient http;

  //String address = "http://192.168.22.21/index_php.php";
  //String address = "http://192.168.22.21/formtest.html";
  String address = "http://192.168.44.100/formtest.html";

  //いらんかもしれないdelay
  delay(500);
  Serial.println(address);

  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");


//  writeReg(0xF2, ctrl_hum_reg);
//  writeReg(0xF4, ctrl_meas_reg);
//  writeReg(0xF5, config_reg);
//  readTrim();                    //
  delay(100);
  Serial.println("pi_client_test");

  setupWiFi();
  Ethernet.begin(MAC, IP);
  server.begin();


  Serial.println(Ethernet.localIP());

 // Serial.println( milkcocoa.on(MILKCOCOA_DATASTORE, "push", onpush) );
};

void loop() {
  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  signed long int temp_cal;
  unsigned long int sleep, sec, msec;           //int型だけだとマイクロ秒で1時間できない
  unsigned long int press_cal, hum_cal;

  //String address = "http://192.168.22.21/index_php.php";
  //↓入力フォームと同じ動作で、直接test1.phpを動作させる
  String address = "http://192.168.44.100/filerw.php";
//String address = "http://192.168.22.21/test1.php";

// 熱電対の動作
  // basic readout test, just print the current temp
   Serial.print("Internal Temp = ");
   Serial.println(thermocouple.readInternal());

   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = ");
     Serial.println(c);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFahrenheit());

   delay(1000);
//ここまで熱電対の場合

  readData();

  // 温度取得
  temp_cal = calibration_T(temp_raw);
  // 気圧取得
  press_cal = calibration_P(pres_raw);
  // 湿度取得
  hum_cal = calibration_H(hum_raw);

  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  hum_act = (double)hum_cal / 1024.0;

  Serial.print("TEMP : ");
  Serial.print(temp_act);
  Serial.print(" DegC  PRESS : ");
  Serial.print(press_act);
  Serial.print(" hPa  HUM : ");
  Serial.print(hum_act);
  Serial.println(" %");

//  DataElement elem = DataElement();
//  elem.setValue("TEMP", temp_act);
//  elem.setValue("PRESS", press_act);
//  elem.setValue("HUM", hum_act);
//  milkcocoa.push(MILKCOCOA_DATASTORE, &elem);
  delay(1000);

  //####httpclientでデータ送信()###


     const char* host = "192.168.44.99";
//   const char* host = "192.168.22.21";
     address += "?temp=";
     address += String(temp_act);
     address += "&hum=";
     address += String(hum_act);
     address += "&press=";
     address += String(press_act);
     char buf[100];
     int len = address.length();
     address.toCharArray(buf,len);
//   getpage(buf);    よくわからんので外す

    Serial.println(" send URL");
    Serial.println(address);
    Serial.println ("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
 String getpage(char host[]);

    HTTPClient http;

    http.begin(address);

    int httpadd = http.GET();

    String result = "";

      if (httpadd < 0) {
    result = http.errorToString(httpadd);
  } else if (http.getSize() < 0) {
    result =  "size is invalid";
  } else {
    result = http.getString();
  }

  Serial.println(result);
  Serial.println("ここまでhttp");

  http.end();

  //return result;


  delay(500);

    EthernetClient client = server.available();
  if (client == true) {
    // 受信した1バイトを送り返す
    Serial.println(client.read());
  }

  delay(500);
  //DEEP SLEEPモード突入命令
  Serial.println("DEEP SLEEP START!!");

  //1:μ秒での復帰までのタイマー時間設定  2:復帰するきっかけの設定（モード設定）
  //int型だけだと(2^32-1μ秒)で最大21分くらいしか設定できない
  //unsigned int でも42分
  sec = (1000);
  msec = (1000);
  sleep = (4000);

  ESP.deepSleep(sleep * sec * msec , WAKE_RF_DEFAULT);

  //deepsleepモード移行までのダミー命令
  delay(1000);
}

void readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }

    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;

    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();
}


void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);
}
