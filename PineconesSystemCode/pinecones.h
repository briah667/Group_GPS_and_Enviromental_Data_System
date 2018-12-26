#pragma once
/************************************************************************


//This system is implemented as a function queue scheduling RTOS


  PINOUT:
  AIRQUA
  A0: Airquality Data (yellow) ; Airqual sensor goes to 5V and ground

  LCD
  pin 3: LCD RX ; lcd also connects to 5V and ground--connect to soft serial TX (pin 3 )

  GPS
  pin 7 1pss pin (yellow wire)
  pin 6 3df		 (orange wire)
  also uses 3.3 V and ground
  make sure to connect rx and tx to Serial1 arduino pins  18  and 19 for Tx1 and Rx1 (pmod terminals: rx on gps = blue --> TX1 pin (18), tx on gps  = gray --> RX1 (19)


  BARRO
  5v, ground
  on mega: use top  SDA and SCL pins closest to usb port
 

  BUTTON
  use 2 pins: on bottom connect to 5V, and 10K resistor to ground. on same terminal as 10K resistor, connect button output
  that goes high when button is pressed
  Connect pin 2 (orange wire) to the resistor terminal


  ACCELEROMETER
  connect top left of 12 pin header to 3.3V, 3rd pin from top left to ground;
  connect 4 pins on top header to pins to SDA (brown wire), SCL  (white wire) (use 20 and 21), Power and Ground:

  BUZZER
  Pin 53 (blue wire) connects through resistor to positive terminal, neg to gnd

  LED
  connects to Pin 10 (white wire), resistor to gnd

 */


#include <SoftwareSerial.h>

//LCD PINS
#define LCDRX 4
#define LCDTX 3

//GPS PINS
#define _3DFpin   6 //pin 40, JF-01
#define _1PPSpin  7 //pin43, JF-04
#define DEG2RAD  180*PI;

//GPS state variables
typedef enum {
	RESTART,
	PREFIXED,
	NOTFIXED,
	FIXED,
	PERIPHERALS
}STATE;

typedef enum {
	DEGREES,
	MINUTES,
	SECONDS,
}GPSDATA;
char* LAT;
char* LONG;
#define PI 3.1415926535897932384626433832795
String inStringLat = "", inStringLong = "";
float coordDegreesLong, coordDegreesLat, coordDegrees;
float minutesToDegrees = 0.00000, secondsToDegrees = 0.00000;
String referenceLatitude;
String referenceLongitude;
String currentLatitude, currentLongitude;
float DDcurrentLatitude, DDcurrentLongitude;
float DDreferenceLatitude, DDreferenceLongitude;
float SeattleEarthRadius = 6366518; //in m

float SeattleLatitude = 47.6062;
float SeattleLongitude = 122.3321;
float snohoLat = 47.7950417;
float snohoLong = 122.065667;

float latDiff, longDif;
float directionDegrees;
float directionMagnitude;



//AIR QUAL PIN
#define AIRQUAL A0
volatile int current_AIR_quality = -1;
//int airReadings[30]; //keep samples from past 5 min  (6 samples/min at 10 sec interval)
int airIndex = 0; //array index
int pollutionCount = 0;
 


//Barometer Setup
float baroPascals, baroAltm, baroTempC;
int pressureIndex = 0;
int altIndex = 0;
int tempIndex = 0;
const float ALT_TOLERANCE = 100;
volatile float COMPANION_ALTITUDE=0; //STORAGE FOR OTHER SENSOR DATA INCOMING VIA WIFI



//Sensor Sampling Constants
const long BARRO_SAMPLING = 5000; //milliseconds
const long AQ_SAMPLING = 10000;
//const long GPS_SAMPLING = 2000;
const long ACCEL_SAMPLING = 5000;
long timeElapsedB, timeElapsedAQ, timeElapsedGPS, timeElapsedACCEL;
 


//BUTTON, LED, BUZZER setup
#define BUTTON 2 //must be an interrupt pin--2 is ok on both mega and uno
volatile bool pressed = false;

#define LED  10 //connect with a 330 ohm resitor
#define BUZZER 51







//accelerometer setup
// Déclaration of the adress of the module
#define ADXL345_Adresse 0x53 // ADXL345 adress
#define POWER_CTL 0x2D // Power Control register
#define DATA_FORMAT 0x31 // Data Format register
#define DATAX0 0x32 // LSB axe X
#define DATAX1 0x33 // MSB axe X
#define DATAY0 0x34 // LSB axe Y
#define DATAY1 0x35 // MSB Y
#define DATAZ0 0x36 // LSB axe Z
#define DATAZ1 0x37 // bMSB Z

// Configuration of the module
#define ADXL345_Precision2G 0x00
#define ADXL345_Precision4G 0x01
#define ADXL345_Precision8G 0x02
#define ADXL345_Precision16G 0x03
#define ADXL345_ModeMesure 0x08




