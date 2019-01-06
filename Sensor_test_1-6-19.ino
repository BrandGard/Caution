#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Encoder.h>

#define addressPH 99             //default I2C ID number for EZO pH Circuit.
#define addressDO 97             //default I2C ID number for EZP DO Circuit.
#define addressORP 98            //default I2C ID number for EZP ORP Circuit.
#define addressTEMP 102

byte code=0;                     //used to hold the I2C response code. 
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
char do_data[20];                //we make a 20 byte character array to hold incoming data from the DO circuit. 
char orp_data[20];               //we make a 20 byte character array to hold incoming data from the ORP circuit.
char temperature[20];            //we make a 20 byte character array to convert the incoming temp data to a char array.

byte in_char=0;                  //used as a 1 byte buffer to store in bound bytes from the pH Circuit.   
byte i=0;                        //counter used for ph_data array. 
int time=1400;                   //used to change the delay needed depending on the command sent to the EZO Class pH Circuit. 

float ph_float;                  //float var used to hold the float value of the pH. 
float do_float;                  //float var used to hold the float value of the do.
float orp_float;                 //float var used to hold the float value of the ORP.
float temp;                      //Stores temp received from probe to be converted to a char array.

//GPS Module
Adafruit_GPS GPS(&Serial2);  // Arduino MEGA Hardware Serial Port #2c        
#define GPSECHO true      // If true, GPS sentences are displayed in the Serial Monitor
#define LOG_FIXONLY true  // if true, data will only be recorded when there is a GPS fix
#define GPSRATE 9600      // Default baud rate for adafruit Ultimate GPS module

Encoder myEnc(2,3);
int encoderValue;
int pinInput1 = 47;
int pinInput2 = 49;

//SD card Init
File logfile;

#define HEADER "Date,Time,Latitude,Longitude,pH,ORP,DO,TEMP (C)"//,Month,Day,Year,Hour,Minute"
int csPin = 53;
char filename[15];

void setup() {
 pinMode(pinInput1, OUTPUT);
 pinMode(pinInput2, OUTPUT);
 Serial.begin(115200);
 Wire.begin();

 void count(void);         // code for counting the increasing values of encoder ticks void setup()
  encoderValue = 0;
 
 pinMode(3,INPUT);                       //Pin 3 to read encoder
  attachInterrupt(2,count,FALLING);

  pinMode(csPin, OUTPUT);
  createLogFile();
  initGPS();

}

void loop() {
  getGPS();
  getAtlas(); 
  
  digitalWrite(pinInput1, HIGH);
  digitalWrite(pinInput2, LOW);
  Serial.println(myEnc.read());
  delay(100);
  
  //delay(5000);*/
}
void getData(const int address, char data [], float convertData){
  Wire.beginTransmission(address);    //call the circuit by its ID number.  
  Wire.write('r');                    //transmit the command that was sent through the serial port.
  Wire.endTransmission();               //end the I2C data transmission. 
  delay(time);                          //wait the correct amount of time for the circuit to complete its instruction. 
  Wire.requestFrom(address,20,1);     //call the circuit and request 20 bytes (this may be more then we need).
  code=Wire.read();                     //the first byte is the response code, we read this separately.  
   
  while(Wire.available()){              //are there bytes to receive.  
    in_char = Wire.read();             //receive a byte.
    data[i]= in_char;               //load this byte into our array.
    i+=1;                              //incur the counter for the array element. 
    
    if(in_char==0){                  //if we see that we have been sent a null command. 
      i=0;                          //reset the counter i to 0.
      Wire.endTransmission();       //end the I2C data transmission.
      break;                        //exit the while loop.
    }
  }
  convertData=atof(data); 
  //Serial.println(data);
}

void sendTemp(const int address)
{
  Wire.beginTransmission(address);
  Wire.write("T,");
  Wire.write(temperature);
  Wire.endTransmission();
  delay(300); 
}
void getAtlas(){ //  Serial.print("Getting Atlas");
  
  for(int i=0; i<=4; i++){
    getData(addressTEMP, temperature, temp);
//     temperature = atof(temperataure);
   // temperature = (temperature*(1.8))+32;
  }
  
  sendTemp(addressPH);
  sendTemp(addressDO);
  
  for(int i=0; i<=4; i++){
    getData(addressPH, ph_data, ph_float);          //gets the ph data.
    getData(addressORP, orp_data, orp_float);       //gets the opr data.
    getData(addressDO, do_data, do_float);          //gets the do data.
   
  }

  logfile.print(ph_data);
  logfile.print(",");
  logfile.print(orp_data);
  logfile.print(",");
  logfile.print(do_data);
  logfile.print(",");
  logfile.print(temperature);
  logfile.print(",");

  logfile.println("");
  logfile.flush(); 

  Serial.println("Got Atlas");
  Serial.print("pH:   "); Serial.println(ph_data);
  //Serial.print(",");
  Serial.print("ORP:  "); Serial.println(orp_data);
  //Serial.print(",");
  Serial.print("DO:   "); Serial.println(do_data);
  //Serial.print(",");
  Serial.print("TEMP: "); Serial.println(temperature);
  Serial.println();
}
  void count()
{
  encoderValue++;
}
// SD Card  --------------------------------------------------------

void error(uint8_t errornum){
  while(1){         // Infinite loop
    switch(errornum){
      case 1:  // SD card initialization failure
        
        
        Serial.println("Card Init Failed!");  // ~ for debugging
        Serial.println("- Card inserted?");   // ~ for debugging
        Serial.println("- Wiring correct?");  // ~ for debugging
        Serial.println("- CS Pin correct?");  // ~ for debugging
        
        break;  
      case 2: // Log file could not be opened on SD card
       
        
        Serial.println("Could Not Create Log!");  // ~ for debugging
        Serial.println("- DATLOG99 reached?");    // ~ for debugging
        Serial.println("- Is card full?");        // ~ for debugging
        Serial.println("- Is card locked?");      // ~ for debugging
        
        break;
    }
    delay(4000);  // Allow time to read error statement 
    Serial.println("Fix error and reset");  // ~ for debugging
    delay(2000);  // Allow time to read error statement
  }
}



void createLogFile(){

  Serial.println("Creating New Log...");  // ~ for debugging
  delay(500);
  
  if(!SD.begin(csPin)){             // Determine if the SD card is present and can be initialized
    error(1);
  }
  
  // Check the SD card for the existence of DATLOG00 up to DATLOG99; stops at the first unused instance
  strcpy(filename, "DATLOG00.CSV");  // Filename (not including extension) can be at most 8 characters
  for(uint8_t i=0; i<100; i++){
    filename[6] = '0' + i/10;   
    filename[7] = '0' + i%10;    
    if (!SD.exists(filename)){   
      break;
    }
  }
  
  logfile = SD.open(filename, FILE_WRITE);  // Create new DATLOG## file
  if(!logfile){
    error(2);
  }

  Serial.print(filename);      // Indicate file was created on LCD display
  Serial.println(" Created");  // ~ for debugging
  delay(1000); 
  
  logfile.write(HEADER);        // Add HEADER as the first line in the DATLOG file
  logfile.println("");
  logfile.flush();              // Ensures any bytes written to the file are physically saved to the SD card 
  
                                // LCD, audio, and Serial notifications

}
//  GPS  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void initGPS(){
  GPS.begin(GPSRATE);
  //Request both RMC & GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Alternatively "(..._OUTPUT_RMCONLY)"
  Serial.println("OUTPUT_RMC");
  // Set the update rate to 1 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Update rate can be 1, 5, or 10 Hz
  Serial.println("UPDATE_1hz");
  // Turn off updates on antenna status (no antenna used for this version of boat)
  GPS.sendCommand(PGCMD_NOANTENNA);
  Serial.println("last Command");     

//  buzz();
}




void getGPS(){
  // Recorded sensor data has no useful meaning without respective GPS coordinates. For this reason, the program will enter
  // an infinite loop until a GPS sentence and fix are successfully obtained. See Footnote 5.
  while(1){   
    char c = GPS.read();  
    if(GPS.newNMEAreceived()){        
      if(!GPS.parse(GPS.lastNMEA())) continue;   // If the last GPS sentence could not be parsed, return to beginning of loop
      // If logging data only when there is a GPS fix AND the last GPS sentence has Active status code, break out of loop.
      if(LOG_FIXONLY && GPS.fix) Serial.print("Got GPS"); break;          
      //else{
      //  lcd.setCursor(0,1); lcd.print("Wait for GPS fix..."); lcd.blink();
      //}
    }
  }
  
  // Convert parsed latitude and longitude from Decimal Minutes format into Decimal Degrees for subsequent calculations. 
  // Note: the modf function is used to separate float values into integer and decimal values. 
  double latB10 = GPS.latitude/100;   // GPS.latitude is a float value
  double lonB10 = GPS.longitude/100;  // GPS.longitude is a float value (@@@@ Lon can be 3-digits: /100 still works? @@@@)
  double intPart, decPart;            // variables for integer and decimal parts
  
  decPart = (modf(latB10, &intPart))/(0.60);  // the 0.60 converts the decimal minutes into hours
  latB10 = intPart + (decPart);
  decPart = (modf(lonB10, &intPart))/(0.60);  // the 0.60 converts the decimal minutes into hours
  lonB10 = intPart + (decPart);
  if(GPS.lat == 'S') latB10 = -latB10; 
  if(GPS.lon == 'W') lonB10 = -lonB10; 
  
  // Log GPS data to SD card
  
  Serial.print("Data Logging...");
  logfile.print(GPS.month); logfile.print("/"); // Date
  logfile.print(GPS.day); logfile.print("/"); 
  logfile.print(GPS.year); 
  logfile.print(",");
  
  logfile.print(GPS.hour); logfile.print(":"); // Time
  logfile.print(GPS.minute); logfile.print(":"); 
  logfile.print(GPS.seconds),2; 
  logfile.print(",");
  
  logfile.print(latB10,6); // Location
  logfile.print(","); 
  logfile.print(lonB10,6);   
  logfile.print(","); 
  
//  logfile.print(GPS.altitude); // Altitude
//  logfile.print(",");
  // If new waypoint reached, record the data and allow for other data to be recorded on the same line; otherwise, "println".
  //if(wptReached == true){
  //  logfile.print(GPS.satellites); logfile.print(",");                  // Satellites
  //}
  //else logfile.println(GPS.satellites);  
  //logfile.flush(); // Ensures data is written onto card  

  // ~ Entire block for debugging
  // Serial.println(GPS.lastNMEA());
  Serial.print("Data Printing");
  Serial.print(GPS.month); Serial.print("/"); // Date
  Serial.print(GPS.day); Serial.print("/"); 
  Serial.println(GPS.year); 
  //Serial.print(",");
  Serial.print(GPS.hour); Serial.print(":"); // Time
  Serial.print(GPS.minute); Serial.print(":"); 
  Serial.println(GPS.seconds); 
  //Serial.print(",");
  
  Serial.print(latB10,4); // Location
  Serial.print(","); 
  Serial.println(lonB10,4);  
  Serial.println(); 
 // Serial.print(",");
  //Serial.print(GPS.speed); Serial.print(","); Serial.print(GPS.angle); // Speed, Angle
  //Serial.print(","); 
  //Serial.print(GPS.altitude); Serial.print(",");  // Altitude,
  //Serial.println(GPS.satellites);               // Satellites
  //buzz();
}




// "parseHex" function obtained from Adafruit_GPS library
uint8_t parseHex(char c){
  if (c < '0') return 0;
  if (c <= '9') return (c - '0');
  if (c < 'A') return 0;
  if (c <= 'F') return (c - 'A') + 10;
} 
