/************************************************************************
*
* Group GPS and Local Environmental Data Tracking System 
*
*************************************************************************
* Author: Ian Etheridge, Sam Stevens, Andrew Steeble, Michle Alem - Autum 2018
*         UW Bothell Sensors and Sensor Systems class, B EE 484
*         GPS code adapted from PmodGPSdemo by Thomas Kappenman
*           Copyright 2014, Digilent Inc.
*           Revised by TommyK 2014
* 
* Description: 
* This code gathers data from the GPS sensor to report local latitude, longitude, and alittude coordinates 
*   including Speed, and differential measurements between systems. 
* The code is normalized for the greater Seattle area and must be adjusted to use further than 100km from Seattle.  
* The code also does not take into account the curvature of the Earth and should not be used for distances greater than 100 km from normalized location.
* An additional antenna can be purchased to increase signal gain, would also need to purchase a component to solder onto the PmodGPS to attatch antenna
* Temperature, barometric pressure, and air quality are also reported.
* All sensors and computation take place on the Arduino Mega controller. This is interfaced with a Raspberry Pi to transmit/recieve data with the other system on the Cloud via Python. 
* Code functionality based on example codes provided for each sensor. 
* Systems made portable by powering Arduino from Pi and Pi from USB connected battery. 
* The name PineCones was a tounge in cheek thing that stuck with us through the project. Naming has not been changed to avoid possible errors. 
* The Arduino connects to the Raspberry Pi via Serial Port TX/RX
*
* Materials:
* 1. Arduino Mega microcontroller
* 2. Raspberry Pi
* 3. PmodGPS - Digilent GPS peripheral module
* 4. Pmod CLS (Jumpers on positions MOD0 and MOD2), (obsolete product, can modify for other displays)
*    See instructions list on https://reference.digilentinc.com/pmod/pmod/cls/user_guide
* 5. MPL115A2 - I2C Barometric Pressure/Temperature Sensor
* 6. Seeed Studio - Grove Air quality sensor v1.3
* 7. Male ro Female fly wires.
************************************************************************/
 

// libraries
#include "pinecones.h"
#include <SoftwareSerial.h>
#include "PmodGPS.h"
#include "AirQuality.h"
#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_MPL3115A2.h"
#include "HardwareSerial.h"

//connect tx pin on lcd to pin PWM pin 3 on arduino
SoftwareSerial lcd(LCDRX, LCDTX);

//pin definitions
#define _3DFpin   6 //pin 6
#define _1PPSpin  7 //pin 7
//#define RSTpin    reset //

//create objects
GPS myGPS;
NMEA mode;
AirQuality AQ;
Adafruit_MPL3115A2 BARROMETER = Adafruit_MPL3115A2();

//state machine states declarations
STATE state=RESTART;
GPSDATA gpsdata = DEGREES;

// Variable initialization of foreign GPS coordinates
float theirLat = 0;
float theirLon = 0;

//Count for foregin device location update
int countdown = 2;

// Blank buffer string to process incomeing serial communication
String in = "";

// Millis timer to break serial read 
long previousMillis = 0;
long interval = 5000;


//sampling timers
long oldBarroTime = millis();
long oldAQTime = millis();
long oldGPSTime = millis();
long oldACCELTime = millis();
long currentTime;
//accelerometer
byte buffer[6]; // storage of data of the module
int j = 0;
int composante_X;
int composante_Y;
int composante_Z;

//helper functions
void screenClear() {  //resets the LCD display to a fresh window w/cursor in 1st line, 1st column
  lcd.write("\x1b[j"); // Erase display
  lcd.write("\x1b[0h"); // configuration of the display (write on 2 lines)
  lcd.write("\x1b[0;0H"); // cursor is on line 1 and columm 1
}

void alarmISR() {
  digitalWrite(LED, HIGH);
  tone(BUZZER, 750);
  delay(1000);
  digitalWrite(LED, LOW);
  noTone(BUZZER);
  delay(500);

}


    /*
    This is the start up loop of the pinecones system. The system will load up all of
    the required devices and initialize required lines. The user will also be promted 
    to wait for the startup sequence to finish.
    */
void setup()
{
//    alarmISR(); //here to alert to crash reset
    lcd.begin(9600); // Begin LCD
    screenClear();
    lcd.print("Starting up...");
    lcd.write("\x1b[1;0H");
    lcd.print("Please Wait");
  
    Serial.begin(9600); // Begin serial communication
    pinMode(AIRQUAL, INPUT);
    AQ.init(AIRQUAL);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
  
    //accelerometer code
    Wire.begin (); // initialization of I2C communication
    Wire.beginTransmission (ADXL345_Adresse); // configuration of the module
    Wire.write (DATA_FORMAT);
    Wire.write (ADXL345_Precision4G);
    Wire.endTransmission ();
    Wire.beginTransmission (ADXL345_Adresse);
    Wire.write (POWER_CTL);
    Wire.write (ADXL345_ModeMesure);
    Wire.endTransmission ();
    
    myGPS.GPSinit(Serial1, 9600, _3DFpin, _1PPSpin); // GPS initialization
    screenClear();
    lcd.print("System is ready");
    delay(3000);
}

    /*
    This code will start the primary state machine which will
    cycle through the sensor data. Data will be visually displayed
    to the user in a way which is appropriate to understand. The code
    will also upload this devices GPS coordinates to the cloud.
    */
void loop()
{
  //State machine for GPS
  switch (state)
  {
    case(RESTART):
        lcd.write("\x1b[1;0H");
        lcd.print("Begin GPS Cycle");
        state = NOTFIXED;
        delay(3000);
        screenClear();       
    break;       
    case(NOTFIXED)://Look for satellites, display how many the GPS is connected to
      mode = myGPS.getData(Serial1);//Receive data from GPS
      if (mode == GGA){//If GGAdata was received
        
        // This will reset the screen and display the number of satelites in a NON FIXED STATE
          if( myGPS.getNumSats() >= 0){
            screenClear();
            lcd.write("\x1b[0;0H");lcd.print("GPS is Not Fixed");
            delay(3000);
            gpsCycle();
            BARRO();
            AirQual();
            gpsToCloud();
          }else{
            screenClear();
            lcd.print("Acquiring Data  From Satelites");
            delay(3000);
          }
                 
        if (myGPS.isFixed()){//Position Fix Indicator (PFI) bit is high, not sure what that actually means
          state=FIXED;
        }
      }
    break;

    case(FIXED): //I am still unsure what Posisition Fixed Indicator (PFI) is used for / significance
                 //this code didn't seem to perform differently bewteen NOTFIXED and FIXED
        if(myGPS.isFixed()){//Update data while there is a position fix
          mode = myGPS.getData(Serial1);
          if (mode == GGA)//If GGAdata was received
          {
          screenClear();
          lcd.write("\x1b[0;0H");lcd.print("GPS is Fixed");
          delay(3000);
          gpsCycle();
          BARRO();       
          AirQual();
          gpsToCloud();         
          }
        }
        else {
          state=RESTART;//If PFI = 0, re-enter connecting state
        }
    break;
  }
}
//*******************************************************************************************************************************************
//**************************************************END OF MAIN LOOP*************************************************************************


// functions for loop code, could go into header file


///**************************************************/
///* function: convertDMStoDDlatitude                                                */                                */
///* input: String
///* output: float                                  
///* description: parses string for numerical values
///*   gps coordinates captured in degrees minutes seconds mode
///*   converts DMS format to Decimal Degrees format for calcualtions
///*   convert by dividing minutes data by 60 and seconds data by 3600 then summing each with the degrees data
///*   1 degree is 60 minutes and there are 60 seconds in each minute
///**************************************************/
float convertDMStoDDlatitude(String lat){
          float coordDegreesLat = 0.00;
          String inStringLat = "";

            switch (gpsdata) {
              
              case (DEGREES):
                inStringLat += lat.substring(0,2); //first two chars are degrees
                coordDegreesLat += inStringLat.toFloat(); //convert to float, add to decimal degrees value
                inStringLat = "";//clear temp string
                gpsdata = MINUTES;//move to parse next part of string
               
               case (MINUTES):
                inStringLat += lat.substring(3,5); //skip char, next two are minutes
                minutesToDegrees = (inStringLat.toFloat() / 60);  //convert to float
                coordDegreesLat += minutesToDegrees; //add to decimal degrees value
                inStringLat = "";//clear temp string
                gpsdata = SECONDS;//move to parse next part of string
               
               case (SECONDS):
                inStringLat += lat.substring(6,11); //skip char, last five are seconds with a decimal between two integers
                secondsToDegrees = (inStringLat.toFloat() / 3600);//convert to float
                coordDegreesLat += secondsToDegrees; //add to decimal degrees value
                inStringLat = "";//clear temp string
                gpsdata = DEGREES;//move to parse next part of string

               if (lat[lat.length()-1] == 'S') {
                coordDegreesLong = -coordDegreesLong;
                }
            }
 return coordDegreesLat;
}

///**************************************************/
///* function: convertDMStoDDlongitude                                                */                                */
///* input: String
///* output: float                                  
///* description: parses string for numerical values
///*   gps coordinates captured in degrees minutes seconds mode
///*   converts DMS format to Decimal Degrees format for calcualtions
///*   convert by dividing minutes data by 60 and seconds data by 3600 then summing each with the degrees data
///*   1 degree is 60 minutes and there are 60 seconds in each minute
///*   NOTE: seconds to degrees accuracy improved when normalizing difference in longitude degrees to Seattle area
///**************************************************/
float convertDMStoDDlongitude(String longit){
    
    float coordDegreesLong = 0.0;
    String inStringLong = "";
  
            switch (gpsdata) {
              
              case (DEGREES): //first 3 characters are degrees
                inStringLong += longit.substring(0,3); //parse string
               coordDegreesLong += inStringLong.toFloat(); //convert to float, add to decimal degrees value
               inStringLong = ""; //clear temp string
               gpsdata = MINUTES; //move to parse next part of string
               
               case (MINUTES): //skip a char then next two are minutes
                inStringLong += longit.substring(4,6); //parse string
               minutesToDegrees = (inStringLong.toFloat() / 60);  //convert to float 
               coordDegreesLong += minutesToDegrees; //add to decimal degrees value
               inStringLong = ""; //clear temp string
               gpsdata = SECONDS;//move to parse next part of string
               
               case (SECONDS): //skip a char then last five are seconds with a decimal in the middle of four integers
                inStringLong += longit.substring(7,12); //parse string
                secondsToDegrees = cos(SeattleLatitude*PI/180)*(inStringLong.toFloat() / 3600); //convert to float
                coordDegreesLong += secondsToDegrees; //add to decimal degrees value
                inStringLong = ""; //clear temp string
                gpsdata = DEGREES; //reset to beginning of string state for next function call

               if (longit[longit.length()-1] == 'W') {
                  coordDegreesLong = -coordDegreesLong;
               }
            }
 return coordDegreesLong;
  }

///**************************************************/
///* function: spaceBetween                                                                               */
///* input: 4 floats -> longitude and latitude of current posistion and reference posistion
///* output: float                                  
///* description: takes gps coordinates of two locations and calculates distance between them in meters 
///**************************************************/
float spaceBetween(float longitudeLocal, float longitudeOther, float latitudeLocal, float latitudeOther){
      float longDiff = longitudeLocal - longitudeOther;
      float latDiff = latitudeLocal - latitudeOther;
      float longMeterPerDeg = 85390; //Seattle area distance between degrees longitude
      float latMeterPerDeg = 111030; //Seattle area distance between degrees latitude
      float longMeters = longDiff*longMeterPerDeg; //convert differnce of longitude degrees to meters
      float latMeters = latDiff*latMeterPerDeg;//convert differnce of latitude degrees to meters
      float directionMag = sqrt(longMeters*longMeters + latMeters*latMeters); //pythagorean distance formula
  return directionMag;
}  
    /*
    GPS display Function
    */
void gpsCycle(){
    lcd.write("\x1b[j"); 
    lcd.write("\x1b[1;0h"); 
    lcd.print("# of Sats: ");lcd.print(myGPS.getNumSats());
    delay(3000);

    //This will convert the GPS data into floating variables compute distance to the other device
    currentLatitude = myGPS.getLatitude();
    DDcurrentLatitude = convertDMStoDDlatitude(currentLatitude);
    currentLongitude = myGPS.getLongitude();
    DDcurrentLongitude = convertDMStoDDlongitude(currentLongitude);

    //Hard coded Seattle coordinates
//    DDcurrentLatitude = 49.2848;
//    DDcurrentLongitude = -123.0931;
    
    // This will reset the screen and display the users Latitude coordinates
    screenClear();
    lcd.print("Latitude: ");lcd.write("\x1b[1;0H");lcd.print(DDcurrentLatitude, 7);//lcd.print(" Deg");     
    delay(3000);
    screenClear();
    
    lcd.print("Longitude: ");lcd.write("\x1b[1;0H");lcd.print(DDcurrentLongitude, 7);//lcd.print(" Deg");
    delay(3000);
    screenClear();
     
    //This code will check the distance between two points, and display the distance in a user friendly manner
    directionMagnitude = spaceBetween(DDcurrentLongitude, theirLon, DDcurrentLatitude, theirLat);
    if(directionMagnitude/100 > 10) {
      lcd.print("Distance to Ref: ");lcd.write("\x1b[1;0H");lcd.print(directionMagnitude/1000);
      lcd.print(" km");
    }else {
      lcd.print("Distance to Ref: ");lcd.write("\x1b[1;0H");lcd.print(directionMagnitude);
      lcd.print(" m");
    }
    delay(3000);
    directionDegrees = directionToDegrees(DDcurrentLongitude, theirLon, DDcurrentLatitude, theirLat);
    screenClear(); 
    
    lcd.print("Angle: ");lcd.print(directionDegrees*180/PI);lcd.write("\x1b[1;0H");
    lcd.print("Deg: ");lcd.print(directionToCompass(directionDegrees));
    delay(3000);
    screenClear();
     
    lcd.print("Speed: ");lcd.write("\x1b[1;0H");lcd.print(myGPS.getSpeedKM(), 3);lcd.print(" km/hr");
    delay(3000); 

    //there is an altitude function for PmodGPS as well. refer to header files
}
  
    /*
    delivers total exposurd over 5 minute period to user as total number of seconds
    and % of last 5 minutes with warning for exposure beyond 1 minute to low or high pollution levels
    */
void AirQual() {
  if (current_AIR_quality >= 0) {
    screenClear();
    if (current_AIR_quality == 0) {
      lcd.write("High pollution!");
      ++pollutionCount;
      alarmISR();
    }
    else if (current_AIR_quality == 1) {
      lcd.write("High pollution!");
      ++pollutionCount;
      alarmISR();
    }
    else if (current_AIR_quality == 2) {
      lcd.write("Low pollution!");
      ++pollutionCount;
    }
    else if (current_AIR_quality == 3)
      lcd.write("Fresh air");
  }
  delay(1000);
  //reset after 30 cycles
  if (airIndex == 29) {
    airIndex  = 0;
    pollutionCount = 0;
  }
  delay(2000);
}

//air qual default ISR
ISR(TIMER1_OVF_vect) {
  if (AQ.counter == 61) { //set 2 seconds as a detected duty
    AQ.last_vol = AQ.first_vol;
    AQ.first_vol = analogRead(AIRQUAL);
    AQ.counter = 0;
    AQ.timer_index = 1;
    PORTB = PORTB ^ 0x20;
    current_AIR_quality = AQ.slope();
  }
  else {
    AQ.counter++;
  }
}

    /*
    barometer
    barometer is sampling every 5 seconds.
    */
void BARRO() {
  if (!BARROMETER.begin()) {
    lcd.write("Couldn't find BARO sensor");
    delay(3000);
  }
  else {
    baroPascals = BARROMETER.getPressure();
    baroAltm    = BARROMETER.getAltitude();
    baroTempC   = BARROMETER.getTemperature();

    screenClear();
    lcd.write("Pressure: ");
    lcd.print(baroPascals / 3377); //convert to HG
    lcd.write(" inch Hg ");
    delay(3000);

    screenClear();
    lcd.print("Alt: ");
    lcd.print(baroAltm);
    lcd.print(" m");
    delay(3000);

    screenClear();
    lcd.write("Temp: ");
    lcd.print(baroTempC);
    lcd.print(" C " );
    delay(3000);
  }
}

    /*
    This code will cycle through and update the local and foregin
    GPS coordinates. Further code can be added to process location differences
    */   
void gpsToCloud(){

    screenClear();
    lcd.print("Refreshing...");
    lcd.write("\x1b[1;0H");
    lcd.print("Please wait");
    delay(500);
    //Prints the Latitiude with extra decimals for zero loss
    Serial.println(DDcurrentLatitude,13);
    delay(2000); //delay for stability
    //Same as above, but for longitude
    Serial.println(DDcurrentLongitude,13);
  
    unsigned long currentMillis = millis();
    previousMillis = currentMillis;
    
    // This loops waits until valid serial communication is received.
    //Once data has been revieved, the foreign GPS latitude is updated
    while(countdown == 2 && (currentMillis - previousMillis < interval)) {
      currentMillis = millis();
      if(Serial.available() > 0) {
         while(Serial.available() > 0) {
            in = in + Serial.readString();
         }
         theirLat = in.toFloat();
         in = "";
         countdown = 1;
      }
    }
    while(currentMillis - previousMillis >= interval){
      alarmISR();
      screenClear();
      lcd.print("ERROR REBOOT PI");
      delay(5000);
    }
    // Same loop process as above, but with foreign Longitude Data
    while(countdown == 1) {
      if(Serial.available() > 0) {
         while(Serial.available() > 0) {
            in = in + Serial.readString();
         }
         theirLon = in.toFloat();
         in = "";
         countdown = 2;
      }
    }
}

    /*
    determine direction to reference from local coordinates in degrees
    */
float directionToDegrees(float longL, float longO, float latL, float latO){

    float X = cos(latO*PI/180)*sin((longO-longL)*PI/180);
    float Y = cos(latL*PI/180)*sin(latO*PI/180)-sin(latL*PI/180)*cos(latO*PI/180)*cos((longO-longL)*PI/180);
    float directionToDegrees = atan2(X,Y);
 return directionToDegrees;
}

    
    /*
    determine compass direction to reference location given degrees of rotation 
    */
String directionToCompass(float dirDeg){
    
    String dToComp = "";
    float dDeg = dirDeg*180/PI;
    
    if(dDeg > 168.75 || dDeg < -168.75){dToComp = "S"; 
    } else if(dDeg <= 168.75 && dDeg >= 146.25){dToComp = "SSE";
    } else if(dDeg < 146.25 && dDeg > 123.75){dToComp = "SE";
    } else if(dDeg <= 123.75 && dDeg >= 101.25){dToComp = "ESE";
    } else if(dDeg < 101.25 && dDeg > 78.75){dToComp = "E";
    } else if(dDeg <= 78.75 && dDeg >= 56.25){dToComp = "ENE";
    } else if(dDeg < 56.25 && dDeg > 33.75){dToComp = "NE";
    } else if(dDeg <= 33.75 && dDeg >= 11.25){dToComp = "NNE";
    } else if(dDeg < 11.25 && dDeg > -11.25){dToComp = "N";
    } else if(dDeg <= -11.25 && dDeg >= -33.75){dToComp = "NNW";
    } else if(dDeg < -33.75 && dDeg > -56.25){dToComp = "NW";
    } else if(dDeg <= -56.25 && dDeg >= -78.75){dToComp = "WNW";
    } else if(dDeg < -78.75 && dDeg > -101.25){dToComp = "W";
    } else if(dDeg <= -101.25 && dDeg >= -123.75){dToComp = "WSW";
    } else if(dDeg < -123.75 && dDeg > -146.25){dToComp = "SW";
    } else if(dDeg <= -146.25 && dDeg >= -168.75){dToComp = "SSW"; 
    }
 return dToComp;
}
