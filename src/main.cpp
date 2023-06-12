#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <iostream>
#include <string>
// lib pour capteur de température
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//lib pour l'altitude
#include <SPI.h>
#include <Adafruit_BMP280.h>


#define RXD0 16
#define TXD0 17

#define DHTPIN 2
#define DHTTYPE DHT11


HardwareSerial SerialPort(2);
DHT_Unified dht(DHTPIN, DHTTYPE);
int loop_nb = 0;


Adafruit_BMP280 Altimeter;

void setup() {

  //Serial.begin(9600, SERIAL_8N1, RXD0, TXD0);
  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, RXD0, TXD0);
  Altimeter.begin();
  
  
  //Serial.begin();
  /*Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
  Serial.write("test");*/

  dht.begin();
}



//traitement des coordonnées
//convertie la coordonnée qui est sous la forme : 
//                      degrés + minutes + "." + secondes en décimales 
//                                      vers
//                      degrés + "." + minutes + secondes
float convert_latitude_longitude(String to_convert){ 
    //conversion du string en tableau de char  
    char* coordinate = new char[to_convert.length()];
    strcpy(coordinate, to_convert.c_str());
    //conversion de tableau de char en float
    float flt_coordinate = atof(coordinate);

    int firstdigits = ((int)flt_coordinate)/100;
    float lastdigits = flt_coordinate - (float)(firstdigits*100);
    float converted = ((float)(firstdigits + lastdigits/60.00));
    return converted;
}

void gps_read(){  
  String gps_data;
  String gps_time;
  String Latitude;
  String Latitude_f;
  char Latitude_NS;
  String Longitude;
  char Longitude_WE; 
  String Altitude;

  while(gps_data.startsWith("$GPGGA") == false){
    gps_data = SerialPort.readStringUntil(13);
    gps_data.trim();
  }

  int Pos = gps_data.indexOf(',');
  gps_data.remove(0,Pos+1);
  

  //Heure UTC
  Pos = gps_data.indexOf(',');
  gps_time = gps_data;
  gps_time.remove(Pos, 80);
  gps_data.remove(0, Pos+1);
  Serial.print("Heure UTC: " + gps_time + "\n");

  //Latitude
  Pos = gps_data.indexOf(',');
  Latitude = gps_data;
  Latitude.remove(Pos, 80);
  gps_data.remove(0, Pos+1);
  Serial.print("Latitude: ");
  Serial.print(convert_latitude_longitude(Latitude), 6);
  Serial.print(" ");

  //Latitude Nord ou Sud
  Pos = gps_data.indexOf(',');
  Latitude_NS = gps_data.charAt(Pos-1);
  gps_data.remove(0, Pos+1);
  Serial.print(Latitude_NS);
  Serial.print("\n");

  //Longitude
  Pos = gps_data.indexOf(',');
  Longitude = gps_data;
  Longitude.remove(Pos, 80);
  gps_data.remove(0, Pos+1);
  Serial.print("Longitude: ");
  Serial.print(convert_latitude_longitude(Longitude), 6);
  Serial.print(" ");

  //Longitude Ouest ou Est
  Pos = gps_data.indexOf(',');
  Longitude_WE = gps_data.charAt(Pos-1);
  gps_data.remove(0, Pos+1);
  Serial.print(Longitude_WE);
  Serial.print("\n");

  //Altitude 
  Pos = gps_data.indexOf(',');
  gps_data.remove(0, Pos+1);
  Pos = gps_data.indexOf(',');
  gps_data.remove(0, Pos+1);
  Pos = gps_data.indexOf(',');
  gps_data.remove(0, Pos+1);
  Altitude = gps_data;
  Altitude.remove(Pos, 80);
  
  Serial.print("Altitude: " + Altitude + " m");
  Serial.print("\n");
}


void course_and_speed(){
  String gps_data;
  String gps_time;
  String speed;
  String course;
  
  while(gps_data.startsWith("$GPRMC") == false){
    gps_data = SerialPort.readStringUntil(13);
    gps_data.trim();
  }

  int Pos;
  for (int i = 0; i < 7; i++)
  {
    Pos = gps_data.indexOf(',');
    gps_data.remove(0,Pos+1);
  }
  Pos = gps_data.indexOf(',');
  speed = gps_data;
  speed.remove(Pos, 80);
  gps_data.remove(0,Pos+1);
  Pos = gps_data.indexOf(',');
  course = gps_data;
  course.remove(Pos, 80);

  char* chr_speed = new char[speed.length()];
  strcpy(chr_speed, speed.c_str());
  //conversion de tableau de char en float
  float speed_kph = atof(chr_speed);
  speed_kph = speed_kph *1.852;

  Serial.print("Vitesse: ");
  Serial.print(speed_kph,2);
  Serial.print(" km/h\n");
  Serial.print("Direction:" + course + "°\n");
}

//fonction de réception de la température
void temperature_humidity_read(){
  
  uint32_t delayMS;
  sensor_t sensor;
  delayMS = sensor.min_delay / 500;  
  //delay(delayMS);

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }

  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}

float altitude (){
  //Altimeter.begin();
  Serial.print("Atitude barométrique :");
  float alt;
  alt = Altimeter.readAltitude(1015);
  Serial.print(alt);
  Serial.print("\n");
  return alt;
}

int volume (){
  int val = 0;
  for (int i = 0; i < 200; i++)
  {
    if(val<analogRead(32))val = analogRead(32);
  }
  Serial.print("Volume sonore : ");
  Serial.print(val);
  Serial.print("\n");
  return val;

}

void loop() {
  
  //temperature_humidity_read();
  //gps_read();
  //course_and_speed();
  altitude();
  volume();


  Serial.print("\n\n");

  delay(500);
}