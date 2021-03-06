//LIBRARIES
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h> //accelerometer and magnetometer
#include <Adafruit_L3GD20_U.h> //gyroscope
#include <Adafruit_GPS.h> //GPS
#include <Adafruit_BMP280.h> //Barometer & Thermometer
#include <Scheduler.h>
#include <SD.h>

//DEFINE
#define GPSSerial Serial1
#define GPSECHO false
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

//UNIQUE SENSOR ID
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321); //accelerometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345); //magnetometer
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20); //gyroscope
Adafruit_GPS GPS(&GPSSerial); //GPS
Adafruit_BMP280 bmp; //BMP

//CONSTANTS
const float AGCUTOFF = 0.1; // if the measured Gs fall below this value, the apogee is assumed to have been reached.
const float VELCUTOFF = 0.3; // if the measured velocity falls below this value, the end of the flight is assumed to have been reached.
const float SMOOTHING = 0.05; // for smoothing velocity data
const int chipSelect = 10; // for the SD card
const String logFile = "datalog.txt"; // name of file to which data will be written

//GLOBAL VARIABLES
String stage = "launch";
uint32_t timer = millis();
float groundLevelPressure = 1013.25;
float groundLevelAltitude = 10000;
float gVelocity = 0.2; //by maintaining a global velocity value, in-flight data smoothing can be achieved.
bool fCPassed = false; //set to true once below 500m
bool tCPassed = false; //set to true once below 300m
bool tDPassed = false; //set to true once below 30m

//CALCULATE MAGNITUDE OF 3-D FLOAT VECTOR
float tdm(float x, float y, float z){
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

//CONVERT MODE INTO INTEGER CODE FOR SWITCH STATEMENT
int parse_mode(String mode){
  if (mode == "launch")
    return 0;
  if (mode == "descent")
    return 1;
  if (mode == "landing")
    return 2;
}

//LOG TO SD CARD
void log_data(String data){
  File dataFile = SD.open(logFile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
    Serial.println(data);
  } else {
    Serial.println("ERROR OPENING " + logFile);
  }
}

//SENSOR SETUP
void init_sensors()
{
  //SD card stuff
  Serial.println("CHECKING FOR SD CARD:");
  if (!SD.begin(chipSelect)) {
    Serial.println("ERROR CARD FAILED/NOT PRESENT");
    return;
  } else {
    Serial.println("CONNECTION CONFIRMED.");
  }

  // IMU stuff
  log_data("CHECKING FOR ACCELEROMETER:");
  if(!accel.begin())
  {
    log_data("ERROR NO LSM303 DETECTED -- CHECK WIRING");
    while(1);
  } else {
    log_data("CONNECTION CONFIRMED.");
  }
  
  log_data("CHECKING FOR MAGNETOMETER:");
  if(!mag.begin())
  {
    log_data("ERROR NO LSM303 DETECTED -- CHECK WIRING");
    while(1);
  } else {
    log_data("CONNECTION CONFIRMED.");
  }

  log_data("CHECKING FOR GYROSCOPE:");
  if(!gyro.begin()){
    log_data("ERROR NO L3GD20 DETECTED -- CHECK WIRING");
    while(1);
  } else {
    log_data("CONNECTION CONFIRMED.");
  }
  
  //BMP SETUP
  log_data("CHECKING FOR BMP:");
  if (!bmp.begin()) {
    log_data("ERROR NO BMP280 DETECTED -- CHECK WIRING");
    while(1);
  } else {
    log_data("CONNECTION CONFIRMED.");
  }
  groundLevelPressure = bmp.readPressure() / 100;
  
  //GPS SETUP
  log_data("SETTING UP GPS:");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  log_data("DONE.");
}

void change_mode(String newMode){
  log_data("Switching from " + stage + " mode to " + newMode + " mode");
  stage = newMode;
}

void setup() {
#ifndef ESP8266
  while (!Serial); //pauses board until console opens
#endif
  //Serial.begin(9600);
  Serial.begin(115200);
  init_sensors();
  log_data("Current mode: " + stage);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(parse_mode(stage)){
    case 0: //what to do before the rocket has reached apogee, i.e. pre-launch and launch.
    {
      launch_sequence();
    }
    break;
    case 1: //what to do during the descent from apogee, before the ground is reached
    {
      descent_sequence();
    }
    break;
    case 2: //what to do once the ground is reached
    {
      landing_sequence();
    }
    break;
  }
}

void launch_sequence(){
  sensors_event_t event;
  accel.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float totalAcceleration = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  float totalGs = totalAcceleration / 9.81;

  //When apogee is reached, 0Gs are experienced and the Palpatine switches to descent mode
  if (totalGs < AGCUTOFF){
    change_mode("descent");
  }

  //_______________________GPS SEGMENT - LOWEST MEASURED ALTITUDE ASSUMED TO BE BASELINE_________
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) 
      return; 
  }
  if (timer > millis()) timer = millis();
  // approximately every second or so, check current stats
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {
      float alt = GPS.altitude;
      if (alt < groundLevelAltitude)
        groundLevelAltitude = GPS.altitude;
    }
  }
}

void descent_sequence(){
  sensors_event_t event;
  bool velocityStop = false;

  float altitude = 10000;
  
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) 
      return; 
  }
  if (timer > millis()) timer = millis();

  if(millis() - timer > 1000){ //execute every second
    timer = millis(); //reset timer
    //_____________________ACCELERATION SECTION_____________________//
    accel.getEvent(&event);
  
    float xAccel = event.acceleration.x;
    float yAccel = event.acceleration.y;
    float zAccel = event.acceleration.z;
    //note: acceleration provided in ms^-2
    //_____________________END ACCELERATION SECTION_________________//
    //_____________________MAGNETOMETER SECTION_____________________//
    mag.getEvent(&event);
  
    float xMag = event.magnetic.x;
    float yMag = event.magnetic.y;
    float zMag = event.magnetic.z;
    //note: magnetism provided in uT
    //_____________________END MAGNETOMETER SECTION_________________//
    //_____________________GYROSCOPE SECTION_____________________//
    gyro.getEvent(&event);
    
    float xGyro = event.gyro.x;
    float yGyro = event.gyro.y;
    float zGyro = event.gyro.z;
    //note: gyroscope data given in rads^-1
    //_____________________END GYROSCOPE SECTION_________________//
    //_____________________GPS SECTION_____________________//
    String gpsTime = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "." + String(GPS.milliseconds);
    String latitude = "NA";
    String longitude = "NA";
    float velocity = 10000;

    if(GPS.fix) {
      latitude = String(GPS.latitude) + GPS.lat;
      longitude = String(GPS.longitude) + GPS.lon;
      velocity = GPS.speed;
      altitude = GPS.altitude;

      gVelocity += (velocity - gVelocity) * SMOOTHING;
      if (gVelocity < VELCUTOFF){
        velocityStop = true;
      }
    }
    //_____________________END GPS SECTION_________________//
    //_____________________TEMP AND BARO SECTION_____________________//
    float temperature = bmp.readTemperature(); // in °C
    float pressure = bmp.readPressure(); // in Pa
    //_____________________END TEMP AND BARO SECTION_________________//
    
    //_____________________DATAWRITE SECTION_____________________//
    //GPSTime, Accel, xAcc, yAcc, zAcc, Mag, xMag, yMag, zMag, Gyro, xGyro, yGyro, zGyro, GPS, lat, long, alt, vel, BMP, temp, press
    String inputString = gpsTime + ",ACCEL:,"+String(xAccel)+","+String(yAccel)+","+String(zAccel)+
      ",MAG:,"+String(xMag)+","+String(yMag)+","+String(zMag)+
      ",GYRO:,"+String(xGyro)+","+String(yGyro)+","+String(zGyro)+
      ",BMP:,"+String(temperature)+","+String(pressure)+
      ",GPS:,"+latitude+","+longitude+","+String(altitude)+","+String(velocity);
    log_data(inputString);
    //_____________________END DATAWRITE SECTION_________________//
    
    //_____________________TRANSMISSION SECTION_____________________//
    //_____________________END TRANSMISSION SECTION_________________//
  }

  //_________________DETECT 500M_______________________//
  if (bmp.readAltitude(groundLevelPressure) < 500 && !fCPassed){
    fCPassed = true;
    //take 2 photos
  }
  //_________________END DETECT 500M___________________//
  //_________________DETECT 300M_______________________//
  if ((altitude - groundLevelAltitude) < 300 && !tCPassed){
    tCPassed = true;
    //take 2 photos
  }
  //_________________END DETECT 300M___________________//
  //_________________DETECT 30M________________________//
  if ((altitude - groundLevelAltitude) < 30 && !tDPassed){
    tDPassed = true;
    //start video
  }
  //_________________END DETECT 30M____________________//
  //_________________DETECT LANDING____________________//
  if (velocityStop) { 
    //stop video
    change_mode("landing"); 
  }
  //_________________END DETECT LANDING________________//
}

void landing_sequence(){
  //end all things
}

