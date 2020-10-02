// Code for GPS communication and mDot transfer of data to gateway via LoRaWAN OTAA
//Author: Joshua L. Hayes
//jhayesee2020@gmail.com, (270)585-5929
//Function: Takes in information from GPS and Bluetooth then creates a string that is sent over LoRaWAN protical
//          string structer is deviceID,Lattitude,Longitude,HeartRate,Temperature,step count, the commas will 
//          act like a delimiter for the mapping software to know the end of one data and the begining of the next.
//Notes: Bluetooth coding to be added as soon as possible to enable biometric data transfer authored by Edward Ojini 
//       and spliced in by Josh Hayes.

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <RH_Serial.h>
// #include <RHMesh.h>
#include <RHReliableDatagram.h>
#define GATEWAY_ADDRESS 0
#define CLIENT_ADDRESS 2

int siz = 0;
char myLat[10];
char myLon[10];
char myString[50]; //Empty string

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 5
// Connect the GPS RX (receive) pin to Digital 4

SoftwareSerial mySerial2(2, 3); // RX, TX
SoftwareSerial mySerial1(5, 4);

Adafruit_GPS GPS(&mySerial1);

RH_Serial driver(mySerial2);
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

void setup()
{
  //-------------------------------------LoRaWAN------------------------------------------------------------
  // AT Commands for MultiTech
  Serial.begin(9600);
  delay(100);
  Serial.println("Joining Lora Network");
  mySerial2.begin(115200);
  delay(200);
  //Test
  mySerial2.println("AT");
  delay(100);
  //OTAA network ID
  mySerial2.println("AT+NI=1,Ky-Newton"); // Match your network name
  delay(100);
  //OTAA network key
  mySerial2.println("AT+NK=1,Ky-Newton"); // Match your password
  delay(100);
  //frequency sub band 3
  mySerial2.println("AT+FSB=3");
  delay(100);
  //join delay 5 sec
  mySerial2.println("AT+JD=5");
  delay(100);
  //byte rate 53 bytes and distance abt .5-3 mile
  mySerial2.println("AT+TXDR=1");
  delay(100);

  mySerial2.println("AT+NJM=1");
  delay(100);

  mySerial2.println("AT+TXDR=1");
  delay(100);
  //store in memory
  mySerial2.println("AT&W");
  delay(100);
  //reset
  mySerial2.println("ATZ");
  delay(5000);
  //Join network
  mySerial2.println("AT+JOIN");
  delay(1000);
  //-------------------------------------End LoRaWAN------------------------------------------------------------

  // setup delay
  delay(5000);

  //-------------------------------------GPS--------------------------------------------------------------------
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS
  GPS.begin(9600);
  delay(1000);
  // SET OUTPUT FORMAT
  mySerial1.println(PMTK_Q_RELEASE);
  //-------------------------------------END GPS--------------------------------------------------------------------
  mySerial1.listen();
}

uint32_t timer = millis();

// uint8_t recv_buf[RH_MESH_MAX_MESSAGE_LEN];

void loop()
{
  // READ GPS SERIAL AND CHECK FOR NEW GPS LOCATION
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {

    // TRY TO PARSE LAST GPS SENTENCE
    if (!GPS.parse(GPS.lastNMEA())) {
      Serial.println("Warning: failed to parse GPS NMEA message");
      return;
    }
  }

  // SEND GPS FIX EVERY 5 SECONDS
  if (millis() - timer > 5000) {
    // RESET THE TIMER
    timer = millis();

    // IF WE HAVE A 3D FIX, FILL OUT OUTPUT STRING WITH COORDINATES
    if (GPS.fix) {

      // FORMAT MESSAGE
      Serial.println(GPS.latitude, 4);
      Serial.println(GPS.longitude, 4);

      dtostrf(GPS.latitude, 8, 3, myLat);
      dtostrf(GPS.longitude, 8, 3, myLon);

      strcat(myString, "AT+SEND=");
      //Place device ID here
      strcat(myString, "D01,");
      strcat(myString, myLat);
      strcat(myString, ",");
      strcat(myString, myLon);



      // DEBUG THE MESSAGE
      Serial.println(myString);

      // SEND MESSAGE TO LORAWAN MODEM
      // Had to use RHReliableDatagram due to memory constraints, luckily the messages sent behaves much 
      // like the mesh and it should only be important that the stations are aware of the mesh network 
      // since it is single jump, not the nodes.
      if (manager.sendtoWait(myString, strlen(myString), GATEWAY_ADDRESS)) {
        /*
        uint8_t len = sizeof(recv_buf);
        uint8_t from;
        if (manager.recvfromAckTimeout(recv_buf, &len, 3000, &from)) {
          // got reply
        }
        */
      }
      // mySerial2.println(myString);
      memset(myString, 0, strlen(myString));
    }
  }
}
