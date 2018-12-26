/*
This code will cycle through and update the local and foregin
GPS coordinates. Further code can be added to process location differences
*/

// Variable initialization of local and foreign GPS coordinates
float thisLat = 0;
float thisLon = 0;
float theirLat = 0;
float theirLon = 0;


//Count for foregin device location update
int countdown = 2;

// Blank buffer string to process incomeing serial communication
String in = "";

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //Serial.println("Begin:"); // THIS LINE WILL BREAK THE UPLOAD SEQUENCE 
  delay(4000);
}

// the loop routine runs over and over again forever:
void loop() {
  //placeholder for storage of local GPS coordinates
  thisLat = float(-47.4663733); 
  thisLon = float(-122.200321);
 
   //Prints the Latitiude with extra decimals for zero loss
  Serial.println(thisLat,13);
  delay(2000); //delay for stability
  //Same as above, but for longitude
  Serial.println(thisLon,13);
  
  // This loops waits until valid serial communication is received.
  //Once data has been revieved, the foreign GPS latitude is updated
  while(countdown == 2) {
    if(Serial.available() > 0) {
       while(Serial.available() > 0) {
          in = in + Serial.readString();
       }
       theirLat = in.toFloat();
       in = "";
       countdown = 1;
    }
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

  Serial.println(theirLat);
  Serial.println(theirLon);
  delay(5000);        // delay in between reads for stability
}
