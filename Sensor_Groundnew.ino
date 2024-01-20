#include <Arduino.h>
#include <File.h>
#include <SDHCI.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <dht.h>
#include <GNSS.h>
#include <Wire.h>

const long frequency = 433E6;  // LoRa Frequency
const int csPin = 18;          // LoRa radio chip select
const int resetPin = 26;       // LoRa radio reset
const int irqPin = 25;         // LoRa hardware interrupt pin
const int sf = 12;             // Signal Spreading Factor
const int sigbw = 10.4E3;      // Signal bandwidth
const int txPower = 20;        // Transmission power in dB
const int crd = 5;             // Coding rate
const int gain = 5;            // Gain of Power amplifier

#define GasPin A0              // Analog pin for gas sensor
#define SoilPin A1             // Analog pin for Soil moisture sensor
#define dht_pin 2              // Digital pin for temperature humidity sensor

#define beacondur = 2000       // Beacon packets interval
#define datadur = 2000         // Data packets interval

File DataBuff;                 // Data storage buffer in SD card
File BeaconSendBuff;           // Beacon storage buffer in SD card
SDClass SDCard;                // SDCard is an instance of SDHCI class

dht DHT;                       // DHT is an instance of dht class

static SpGnss GNSS;            // Static GNSS intsance for spresense GNSS class 

int Node_no = 0;               // Node ID for sensor-ground node
int counterGNSS = 0;           // Counters for various sensors
int counterDHT = 0;
int countersoil = 0;
int countergas = 0;
int counterData = 0;

String NodeID(int n)           // Converts int node ID to string
{
  int no = 3355 + n;
  String retString = String(no);
  return retString;
}


String GSU_ID = NodeID(Node_no);  // Creates sensor-ground string ID
String Recv_ID = NodeID(1);       // Set receiever node with node ID as 1

int beacon_recv_no = 0;           // No of beacon packets received
int beacon_send_no = 0;           // No of beacon packets sent
int data_recv_no = 0;             // No of data packets received
int data_send_no = 0;             // No of data packets send

int connected_obc[10];            // Connected devices array
int connected_obc_no = 0;         // No of connected devices    

void setup()
{
  Serial.begin(9600);  
  GNSS.begin();                  // Initialize spresense GNSS
  GNSS.start();                  // Initialize serial monitor
  while (!Serial);
  Wire.begin();                  // Initialize I2C communication 
  
  pinMode(LED2,OUTPUT);
  
  LoRa.setSPI(SPI5);             // Initialize loRa with defined specifications  
  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(sigbw);
  LoRa.setTxPower(txPower);
  LoRa.setCodingRate4(crd);
  LoRa.setGain(gain);

  Serial.printf("Ground station:");
  Serial.println(GSU_ID);

  GNSS_Calibrate();                      // Calibrate Spresense GNSS to give location data
  Serial.println("GNSS Calibrated...");
  
  pinMode(LED0, OUTPUT);            
  while(!SDCard.begin())                // Check SD card initialization
    {
      digitalWrite(LED0,HIGH);
      delay(1000);
      digitalWrite(LED0,LOW);
      delay(1000);
    }
      SDCard.remove("Data.csv");
      SDCard.remove("Beacon.csv");
  Serial.println("SD Card initialization success"); 
    
  pinMode(LED1, OUTPUT);                // Check LoRa module initialization
  bool initialized = false;
  Serial.print("Initializing LoRa.");
  while (!initialized) 
    {
        Serial.print(".");
        initialized = LoRa.begin(frequency);
        digitalWrite(LED0,HIGH);
        delay(1000);
        digitalWrite(LED0,LOW);
        delay(1000);  
    }
  Serial.print("Success");
  Serial.println() ; 
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  Beacon_Sender(); 
}

// GNSS calibrate function
//Run while loop until proper location data is obtained from favourable number of satellites 
void GNSS_Calibrate()               
{
 if (GNSS.waitUpdate(-1))
  {
    SpNavData NavData;
    GNSS.getNavData(&NavData);
    digitalWrite(LED2,HIGH); 
    int flg = 0;
    double Lati = NavData.latitude;
    double Longi = NavData.longitude;
    double Alti = NavData.altitude;
    double Latn;
    double Longn;
    double Altn;
    while (1)
    { 
      Latn = NavData.latitude;
      Longn = NavData.longitude; 
      Altn = NavData.altitude; 
      if (Latn != Lati && Longn != Longi && Altn !=Alti)
      {
        digitalWrite(LED2,LOW);
        break;
      }    
    }
  }    
}

// Function to read GNSS data
String GNSSread()
{
  if (GNSS.waitUpdate(-1))
     {
      SpNavData NavData;                     // NavData object stores latitude, longitude, altitude and no of satellite systems
      GNSS.getNavData(&NavData);
      String message = "";
      message += String(NavData.latitude,4);
      message += ",";
      message += String(NavData.longitude,4);  
      message += ",";
      message += String(NavData.altitude,4);
      message += String(counterGNSS);
      counterGNSS++;
      return message;                       // Return string containing location data
    }    
}

// Function to read data from gas sensor
String Gasread()
{
  double GasValue = analogRead(GasPin);  // Read from spresense analog pin for gas sensor 
  String messageGas = "";
  messageGas += "Gas:";
  messageGas += String(GasValue,2);
  countergas++;
  return messageGas;                     // Return string containing gas data
}

String Soilread()
{
  double SoilValue = analogRead(SoilPin)/10;   // Read from spresense analog pin for soil moisture sensor
  String messageSoil = "Moist:";
  messageSoil += String(SoilValue,2);
  messageSoil += "#";
  if (SoilValue < 30)
       messageSoil += "WET";
    else
       messageSoil += "DRY";
  messageSoil += "#";     
  //messageSoil += String(countersoil);
  countersoil++;
  return messageSoil;                         // Read from spresense analog pin for soil moisture sensor
}

String DHTread()
{
 DHT.read11(dht_pin);                         // Read from spresense analog pin for temperature-humidity sensor
 String messageDHT = "";
 messageDHT += "Hum:";
 messageDHT += String(DHT.humidity,2);
 messageDHT += "#";
 messageDHT += "Temp:";
 messageDHT += String(DHT.temperature,2);
 messageDHT += "#";
 counterDHT++;
 return messageDHT;                          // Read from spresense analog pin for temperature humidity sensor
}

  //Create header field for data packets 
  //Data packet<pkttype,DataID,Sendtime,Srcid,Destid,lengthofdata,Data,CounterData>
String Data_Header()
{
  uint64_t times;
  times = millis();
  int timeint = (int)times;
  String timestr = String(timeint);
  int pktType = 2;
  String DataID = String(1); //Sensor node dataID is 1
  String data_head = "";
  data_head += pktType;
  data_head += ",";
  data_head += DataID;
  data_head += ",";
  data_head += timestr;
  data_head += ",";
  data_head += GSU_ID;
  data_head += ",";
  data_head += Recv_ID;
  return data_head;                       // Returns <pkttype,Sendtime,Srcid,Destid> string
}

void loop() 
{
  if (runEvery(beacondur))
  { 
    Beacon_Sender();                     // Creates and sends beacon packets
  }
  //Data:<Humidity#Temperature#Soilmoisture(ppm)#WET/DRY#GasConcentration>
  if (runEvery(datadur))
  {
    String msg = "";                    // Creates and sends data packets
    msg += Data_Header();
    msg += ",";
    String data = String(DHTread());
    data += String(Soilread());
    data += String(Gasread());
    int len = data.length();
    String lenstr = String(len);
    msg += lenstr;
    msg += ",";
    msg += data;  
    msg += ",";
    counterData++;
    msg += counterData++;
    Serial.print("Sensors data packet:");  
    Serial.println(msg);
    DataBuff = SDCard.open("Data.csv",FILE_WRITE); // Stores data in Data.csv file in SD card
    DataBuff.println(msg);
    Serial.println("Data Stored");
    DataBuff.close();
    LoRa_sendMessage (msg);   // Write data packet string to LoRa buffer
  }
}

// Function to set LoRa in receive mode 
void LoRa_rxMode()
{
  LoRa.disableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                        // set receive mode
}

// Function to set LoRa in transmit mode
void LoRa_txMode()
{
  LoRa.idle();                          // set standby mode
  LoRa.enableInvertIQ();               // normal mode
}

// Function to send packets using LoRa
void LoRa_sendMessage(String message) 
{
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

//Function to receive packets using LoRa module   
void onReceive(int packetSize) 
{
  String message = "";
  while (LoRa.available()) 
  {
    message += (char)LoRa.read();
  }
  Serial.print("GSU Receive: ");
  Serial.println(message);
  String msg = String(message);
}

void onTxDone() 
{
  LoRa_rxMode();
}

// Function to set interval for transmitting packets
boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}




// Packet handler function
void Packet_Handler(String msg)
{
  char pkt[20];
  msg.toCharArray(pkt,20);
  char * token = strtok(pkt, ",");           // token points to first value NODEID
  char pkttype = token;                      // pkttype stores NODEID
  if (pkttype == '0')                        // Beacon packet
   {
     beacon_recv_no += 1;                    // Increase received beacon packet counter
     Serial.println("Beacon Packet");
     Beacon_Handler(pkt);                    // Beacon packet handler function
   }
  else if (pkttype == '2')                   // Data packet
   {
     data_recv_no += 1;                      // Increase received beacon packet counter
     Serial.println("Data packet");
     Data_Scan(pkt);                         // Change code here to check data packet to be forwarded or not
   }
}


//Beacon handler function
void Beacon_Handler(String msg)
{
  char pkt[20];
  msg.toCharArray(pkt,20);
  char * tokena = strtok(pkt,","); 
  char pktType = tokena;                    // Get packet type
  tokena = strtok(NULL,",");
  char SendTimea = tokena;                  // Get time at which beacon packet was sent
  tokena = strtok(NULL,",");
  char obc_no = tokena;                     // Get source node of beacon packet
  char pktscan[64];
  int index = 0;
  int flg = 0;
//  BeaconReadBuff = SDCard.open("Beacon.csv", FILE_READ);// Open beacon buffer in read mode
//  if (BeaconReadBuff)
//  {
//    while (BeaconReadBuff.available()) 
//    {
//      pktscan[index] = BeaconReadBuff.read();// <beacon_no,SendTime,SrcID,Latitude,Longitude>
//      if ((pktscan[index] == '\n' || pktscan[index] == '\r'))
//      {
//          pktscan[index] = '\0';
//          index = 0;
//          char sendTimeb; // 
//          char SrcIDb;        
//          if (strlen(pktscan) != 0)
//          {
//            char * tokenb = strtok(pktscan,",");// beacon_no
//            tokenb = strtok(NULL,",");//sendTimeb
//            sendTimeb = atoi(tokenb);
//            
//            tokenb = strtok(NULL,",");//SrcID
//            SrcIDb = tokenb;
//            //Scan the buffer to check whether the packet is already received or not
//            if (obc_no == SrcIDb)
//              {
//                flg++;  
//              }
//          }          
//      }
//      else
//         index++;
//    }
//  }
//  
//  //If there are no packets from obtained obc, store this packet
//  BeaconReadBuff.close();
//  if(flg==0)
//  {
//    Beacon_Store(msg);
//    beacon_send_no++;
//    connected_obc_no++;
//    connected_obc[connected_obc_no-1] = obc_no;
//    int i = 0;
//    Serial.println("conncected OBCs: ");
//    while (i < connected_obc_no)
//    {
//      Serial.print(connected_obc[i]);
//      Serial.print(", ");
//    }
//  }
}

 // Creates beacon packets using GNSS data of Sensor
void Beacon_Sender() // <pkttype, sendtime, SRCID, lat, long, alt, nosat, counter>
{
  Serial.println("Sending beacon packet");
  uint64_t times;
  times = millis();
  int timeint = (int)times;
  String timestr = String(timeint);
  int pktType = 1;
  String Src_ID = GSU_ID;
  String beacon_msg = "";
  beacon_msg += pktType;
  beacon_msg += ",";
  beacon_msg += timestr;
  beacon_msg += ",";
  beacon_msg += Recv_ID;
  beacon_msg += ",";
  beacon_msg += GNSSread();
  beacon_msg += ",";
  beacon_msg += beacon_send_no;
  Serial.print("Beacon packet send:");
  Serial.println(beacon_msg); 
  BeaconSendBuff = SDCard.open("Beacon.csv",FILE_WRITE);
  BeaconSendBuff.println(beacon_msg);
  Serial.println("Beacon Stored");
  BeaconSendBuff.close();
  LoRa_sendMessage(beacon_msg);
  beacon_send_no++;
}
