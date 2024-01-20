#include <Arduino.h>
#include <File.h>
#include <TinyMPU6050.h>
#include <SDHCI.h>
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <GNSS.h>
#include <Wire.h>
#include <stdio.h>  /* for sprintf */
#include <Camera.h>
#include "Zanshin_BME680.h" 

#define   CAM_IMGSIZE_QQVGA_H   (160)
#define   CAM_IMGSIZE_QQVGA_V   (120)
#define   TOTAL_PICTURE_COUNT     (1000)

const long frequency = 433E6;  // LoRa Frequency
const int csPin = 18;          // LoRa radio chip select
const int resetPin = 26;        // LoRa radio reset
const int irqPin = 25;   // change for your board; must be a hardware interrupt pin
const int sf = 12; // Spreading Factor
const int sigbw = 10.4E3;
const int txPower = 20;
const int crd = 5; 
const int gain = 5;

File DataBuff;
File BeaconSendBuff;
SDClass SDCard;

int take_picture_count = 0;
int CamImage::getWidth ();
int CamImage::getHeight ();

MPU6050 mpu (Wire);

static SpGnss GNSS;
BME680_Class BME680; 

int Node_no = 1; // Node ID of LowTier
int counterGNSS = 0; // Counter of GNSS packets 
int counterMPU = 0; // Counter for GYRO packets
int counterimg = 0;
int counterbme = 0;

String NodeID(int n)
{
  int no = 3355 + n;
  String retString = String(no);
  return retString;
}

String Node_ID = NodeID(Node_no);
String GSU_id = NodeID(0);

int beacon_recv_no = 0; // No of beacon packets received
int beacon_send_no = 0;// No of beacon packets sent
int data_recv_no = 0; // No of data packets received
int data_send_no = 0; // No of data packets send

int connected_obc[10]; 
int connected_obc_no = 0;

float altitude(const int32_t press, const float seaLevel = 1013.25)
{
  static float Altitude;
  Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  
void printError(enum CamErr err)
{
  switch (err)
    {
      case CAM_ERR_NO_DEVICE:
        break;
      case CAM_ERR_ILLEGAL_DEVERR:
        break;
      case CAM_ERR_ALREADY_INITIALIZED:
        break;
      case CAM_ERR_NOT_INITIALIZED:
        break;
      case CAM_ERR_NOT_STILL_INITIALIZED:
        break;
      case CAM_ERR_CANT_CREATE_THREAD:
        break;
      case CAM_ERR_INVALID_PARAM:
        break;
      case CAM_ERR_NO_MEMORY:
        break;
      case CAM_ERR_USR_INUSED:
        break;
      case CAM_ERR_NOT_PERMITTED:
        break;
      default:
        break;
    }
}
void CamCB(CamImage img)
{
  if (img.isAvailable())
    {
      img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
    }
  else
    {
      Serial.println("Failed to get video stream image");
    }
}

void setup()
{
  Serial.begin(9600);  
  GNSS.begin();
  GNSS.start();// initialize Spresense GNSS 

  mpu.Initialize();//Initialize MPU
  BME680.begin(I2C_STANDARD_MODE);
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  BME680.setGas(320, 150); 
  Serial.println("BME680 Initialized");
  
  while (!Serial);
  Wire.begin();

  pinMode(LED2,OUTPUT);
  
  LoRa.setSPI(SPI5);
  LoRa.setPins(csPin, resetPin, irqPin);
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth(sigbw);
  LoRa.setTxPower(txPower);
  LoRa.setCodingRate4(crd);
  LoRa.setGain(gain);

  Serial.printf("Node:");
  Serial.println(Node_ID);

  //Camera initialization
  
  CamErr err;
  err = theCamera.begin();
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }  
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }  

  err = theCamera.setStillPictureImageFormat(
     CAM_IMGSIZE_QUADVGA_H,
     CAM_IMGSIZE_QUADVGA_V,
     CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }

      
  //GNSS_Calibrate();
  Serial.println("GNSS Calibrated...");
  
  pinMode(LED0, OUTPUT);
  while(!SDCard.begin())
    {
      digitalWrite(LED0,HIGH);
      delay(1000);
      digitalWrite(LED0,LOW);
      delay(1000);
    }
      SDCard.remove("Data.csv");
      SDCard.remove("Beacon.csv");
      SDCard.remove("Image.csv");
  
      Serial.println("SD Card initialization success"); 

  
  pinMode(LED1, OUTPUT);
  bool initialized = false;
  Serial.print("Initializing LoRa.");
  while (!initialized) 
    {
        Serial.printf(".");
        initialized = LoRa.begin(frequency);
        
          digitalWrite(LED1,HIGH);
          delay(1000);
          digitalWrite(LED1,LOW);
          delay(1000);  
    }
    Serial.print("Success");
    Serial.println();
  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode(); 
  Beacon_Sender(); 
}


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

String GNSSread()
{
  if (GNSS.waitUpdate(-1))
     {
      SpNavData NavData;
      GNSS.getNavData(&NavData);
      String message = "";
      message += Node_ID;
      message += ",";
      message += String(NavData.latitude,4);
      message += ",";
      message += String(NavData.longitude,4);  
      message += ",";
      message += String(NavData.altitude,4);
      message += ",";
      message += String(NavData.numSatellites);
      message += ",";
      message += String(counterGNSS);
      counterGNSS++;
      return message;
    }    
}

String MPUread()
{
  mpu.Execute();
  String message = "";
  message += "Accx:";
  message += mpu.GetAccX();
  message += "#";
  message += "Accy:";
  message += mpu.GetAccY();
  message += "#";
  message += "Accz:";
  message += mpu.GetAccZ();
  message += "#";
  message += "Gyrox:";
  message += mpu.GetGyroX();
  message += "#";
  message += "Gyroy:";
  message += mpu.GetGyroY();
  message += "#";
  message += "Gyroz:";
  message += mpu.GetGyroZ();
  message += "#";
  message += "AAccx:";
  message += mpu.GetAngAccX();
  message += "#";
  message += "AAccy:";
  message += mpu.GetAngAccY();
  message += "#";
  message += "GAnglex:";
  message += mpu.GetAngGyroX();
  message += "#";
  message += "GAngley:";
  message += mpu.GetAngGyroY();
  message += "#";
  message += "GAnglez:";
  message += mpu.GetAngGyroZ();
  message += "#";
  message += "Anglex:";
  message += mpu.GetAngX();
  message += "#";
  message += "Anglex:";
  message += mpu.GetAngY();
  message += "#";
  message += "Anglex:";
  message += mpu.GetAngZ();
  message += "#";
  message += "ACoeff:";
  message += mpu.GetFilterAccCoeff();
  message += "#";
  message += "GCoeff:";
  message += mpu.GetFilterGyroCoeff();
  return message;
}

String Imgread(CamImage img)
{
  String message = "";
      img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
      message += "Size:";
      message += String((int)img.getImgSize());
      message += "#";
      message += "Width:";
      message += String((int)img.getWidth());
      message += "#";
      message += "Height:";
      message += String((int)img.getHeight());
      return message;
}

String BMEread()
{
  static int32_t temp, hum, pres, gas;
  BME680.getSensorData(temp,hum,pres,gas);
  
  float alt = altitude(pres);  
  String message = "";
  message += "Temp:";
  double t = (int8_t)temp/100 + (uint8_t)temp%100;
  double h = (int8_t)hum/1000 + (uint16_t)hum%1000;
  double p = (int16_t)pres/100 + (uint8_t)pres%100 ;
  double g = (int16_t)gas/100 + (uint8_t)gas%100;
  message += String(t);
  message += "#";
  message += "Hum:";
  message += String(h);
  message += "#";
  message += "Pres:";
  message += String(p);
  message += "#";
  message += "Gas:";
  message += String(g);
  return message;
} 
// Data header: <pkttype, Sendtime, SrcID, DestID, Length, Data>
String Data_Header(int dataID) // Returns <pkttype,Sendtime,Srcid,Destid> string
{  
  uint64_t times;
  times = millis();
  int timeint = (int)times;
  String timestr = String(timeint);
  int pktType = 2;
  String DataID = String(dataID);
  String data_head = "";
  data_head += pktType;
  data_head += ",";
  data_head += DataID;
  data_head += ",";
  data_head += timestr;
  data_head += ",";
  data_head += Node_ID;
  data_head += ",";
  data_head += GSU_id;
  return data_head;
}


void loop() 
{
  if (runEvery(2000))
  { 
    Beacon_Sender();
  }
  if (runEvery(2000))
  {
    String MPUmsg = "";
    int dataID = 2;
    MPUmsg += Data_Header(dataID);
    MPUmsg += ",";
    String data = String(MPUread());
    int len = data.length();
    String lenstr = String(len);
    MPUmsg += lenstr;
    MPUmsg += ",";
    MPUmsg += data;  
    MPUmsg += ",";
    counterMPU++;
    MPUmsg += counterMPU;
    Serial.print("MPU Sensor data:");  
    Serial.println(MPUmsg);
    DataBuff = SDCard.open("Data.csv",FILE_WRITE);
    DataBuff.println(MPUmsg);
    Serial.println("Data Stored");
    DataBuff.close();
    LoRa_sendMessage (MPUmsg);
  }
  if (runEvery(2000))
  {  if (take_picture_count < TOTAL_PICTURE_COUNT)
    {
      CamImage img = theCamera.takePicture();
      if (img.isAvailable())
        {
          char filename[16] = {0};
          sprintf(filename, "LT-SHAPE-IoT%03d.JPG", take_picture_count);
          SDCard.remove(filename);
          File myFile = SDCard.open(filename, FILE_WRITE);
          myFile.write(img.getImgBuff(), img.getImgSize());
          myFile.close();
          String Imgmsg = "";
          int dataID = 3;
          Imgmsg += Data_Header(dataID);
          Imgmsg += ",";
          String data = String(Imgread(img));
          int len = data.length();
          String lenstr = String(len);
          Imgmsg += lenstr;
          Imgmsg += ",";
          Imgmsg += data;
          Imgmsg += ",";
          counterimg++;  
          Imgmsg += counterimg;  
          Serial.print("Image data:");  
          Serial.println(Imgmsg);
          DataBuff = SDCard.open("Image.csv",FILE_WRITE);
          DataBuff.println(Imgmsg);
          Serial.println("Data Stored");
          DataBuff.close();
          LoRa_sendMessage (Imgmsg);
        }
    }
  else if (take_picture_count == TOTAL_PICTURE_COUNT)
    {
      theCamera.end();
    }
  take_picture_count++;
  }
  if (runEvery(2000))
  {
    String BMEmsg = "";
    int dataID = 1;
    BMEmsg += Data_Header(dataID);
    BMEmsg += ",";
    String data = String(BMEread());
    int len = data.length();
    String lenstr = String(len);
    BMEmsg += lenstr;
    BMEmsg += ",";
    BMEmsg += data;  
    BMEmsg += ",";
    counterbme++;
    BMEmsg += counterbme;
    Serial.print("BME Sensor data:");  
    Serial.println(BMEmsg);
    DataBuff = SDCard.open("Data.csv",FILE_WRITE);
    DataBuff.println(BMEmsg);
    Serial.println("Data Stored");
    DataBuff.close();
    LoRa_sendMessage (BMEmsg);
  }
}

void LoRa_rxMode(){
  LoRa.enableInvertIQ();                // active invert I and Q signals
  LoRa.receive();                       // set receive mode
}

void LoRa_txMode()
{
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
}

void LoRa_sendMessage(String message)
{
  LoRa_txMode();                        // set tx mode
  LoRa.beginPacket();                   // start packet
  LoRa.print(message);                  // add payload
  LoRa.endPacket(true);                 // finish packet and send it
}

void onReceive(int packetSize)
{
  String message = "";

  while (LoRa.available()) 
  {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
  String msg = String(message);
  Packet_Handler(msg);
//  DataBuff = SDCard.open("Data.csv",FILE_WRITE);
//  DataBuff.println(msg);
//  Serial.println("B");
//  Serial.println("Data Stored");
//  DataBuff.close();
}

void onTxDone() 
{
  LoRa_rxMode();
}

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

//Beacon Packet Store
//void Beacon_Store(String msg)
//{ 
//  BeaconWriteBuff = SDCard.open("/dir/Beacon.csv",FILE_WRITE);
//  BeaconWriteBuff.println(msg);
//  Serial.println("Data Stored");
//  BeaconWriteBuff.close();  
//}

void Data_Scan(String pkt)
{
  
}
// Packet handler function
void Packet_Handler(String msg)
{
  char pkt[20];
  msg.toCharArray(pkt,20);
  char * token = strtok(pkt, ","); // token points to first value NODEID 
  String pkttype = String(token);
  if (pkttype.equals("1")) // Beacon packet
   {
     beacon_recv_no += 1;
     Serial.println("Beacon Packet");
     //Beacon_Handler(pkt);
   }
  else if (pkttype.equals("2")) // Data packet
   {
     data_recv_no += 1;
     Serial.println("Data Packet");
     //Data_Scan(pkt); // Change code here to check data packet to be forwarded or not
   }
}

//Beacon handler function
void Beacon_Handler(String msg)
{
  char Array[100];
  msg.toCharArray(Array,msg.length());
  Serial.println(Array);
  char *pkt[100];
  byte index = 0;
  char *ptr = NULL;
  ptr = strtok(Array,",");
  while(ptr != NULL)
  {
    pkt[index] = ptr;
    index++;
    ptr = strtok(NULL,",");
  }
  Serial.print("Pkttype: ");
  Serial.println(pkt[0]);// Get packet type
  Serial.print("Send time: ");
  Serial.println(pkt[1]); // Get time at which beacon packet was sent
  Serial.print("Source ID: ");
  Serial.println(pkt[2]);// Get source node of beacon packet
  Serial.print("GSU Latitude:");
  Serial.print(pkt[3]);
  Serial.println(" deg");
  Serial.print("GSU Longitude: ");  
  Serial.print(pkt[4]);
  Serial.println(" deg");
  Serial.print("GSU Altitude: ");
  Serial.print(pkt[5]); // Get time at which beacon packet was sent   
  Serial.println(" m"); 
}

void Beacon_Sender() // <pkttype, sendtime, SRCid,lat, long, alt, nosat, counter>
{
  Serial.println("Sending beacon packet");
  uint64_t times;
  times = millis();
  int timeint = (int)times;
  String timestr = String(timeint);
  int pktType = 1;
  String Src_ID = Node_ID;
  String beacon_msg = "";
  beacon_msg += pktType;
  beacon_msg += ",";
  beacon_msg += timestr;
  beacon_msg += ",";
  beacon_msg += Src_ID;
  beacon_msg += ",";
  beacon_msg += GNSSread();
  Serial.print("Beacon packet send:");
  Serial.println(beacon_msg); 
  
    BeaconSendBuff = SDCard.open("Beacon.csv",FILE_WRITE);
    BeaconSendBuff.println(beacon_msg);
    Serial.println("Beacon Stored");
    BeaconSendBuff.close();
  LoRa_sendMessage(beacon_msg);
  beacon_send_no++;
}
