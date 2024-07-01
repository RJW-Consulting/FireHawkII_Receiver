

// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RadioHead.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>
#include "RTClib.h"

#define ADAFRUIT_FEATHER_M0 1

#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4

#define MAX_COMMAND_RETRY 5

#define DRONE_1_ADDRESS 1
#define DRONE_2_ADDRESS 2
#define STATION_ADDRESS 5

#define PACKET_TYPE_FORMAT 'F'
#define PACKET_TYPE_DATA 'D'
#define PACKET_TYPE_COMMAND_RESPONSE 'R'
#define PACKET_TYPE_COMMAND 'C'
#define PACKET_TYPE_PERMISSION_TO_TALK 'T'
#define PACKET_TYPE_FINISHED_TALKING 'X'


int commandRetry = 0;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHReliableDatagram manager(rf95, STATION_ADDRESS);

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

String dataFormat_drone_1 = "";
String dataFormat_drone_2 = "";

String receiverVersion = "Receiver FW Version 1.2 (in process)";

void setDataFormat(uint8_t address, String format)
{
  switch (address)
  {
    case DRONE_1_ADDRESS:
      dataFormat_drone_1 = format;
      break;
    case DRONE_2_ADDRESS:
      dataFormat_drone_2 = format;
      break;
  }
}

bool commandWaiting = false;

#define MAX_PACKET_LENGTH 255
uint8_t packetBuffer[MAX_PACKET_LENGTH];

bool sendPacket(uint8_t to, char packetType, uint8_t *contents, uint8_t length)
{
  if (length > MAX_PACKET_LENGTH-1)
  {
    Serial.println(" ATTEMPT TO SEND OVERSIZED PACKET.");
    return false;
  }
  packetBuffer[0] = (uint8_t)packetType;
  memcpy(packetBuffer+1, contents, length);
  return manager.sendtoWait(packetBuffer, length+1, to);
}

void sendCommand(uint8_t to, String command)
{
  if (!sendPacket(to, PACKET_TYPE_COMMAND, (uint8_t *) command.c_str(), (uint8_t) command.length()+1))
  {
    Serial.print("Send command to drone ");
    Serial.print(to, DEC);
    Serial.print("\"");
    Serial.print(command);
    Serial.print("\"");
    Serial.println(" failed.");
    commandWaiting = true;
    commandRetry--;
  }
  else
  {
    commandWaiting = false;
    commandRetry = 0;
  }
}

void sendFormatStringRequest(uint8_t to)
{
  String command = "df ";
  sendCommand(to, command);
}

DateTime dateTimeFromPacket(uint8_t *data)
{
  char buff[10];
  strncpy(buff, (const char *) data, 4);
  buff[4] = 0;
  String year = buff;
  strncpy(buff, (const char *) data+4, 2);
  buff[2] = 0;
  String month = buff;
  strncpy(buff, (const char *) data+6, 2);
  buff[2] = 0;
  String day = buff;
  strncpy(buff, (const char *) data+8, 2);
  buff[2] = 0;
  String hour = buff;
  strncpy(buff, (const char *) data+10, 2);
  buff[2] = 0;
  String minute = buff;
  strncpy(buff, (const char *) data+12, 2);
  buff[2] = 0;
  String second = buff;
  DateTime time(year.toInt(), month.toInt(), day.toInt(), hour.toInt(), minute.toInt(), second.toInt());

  return time;
}

union typeSort{
  uint8_t uint8;
  uint16_t uint16;
  float flt32;
  uint8_t bytes[4];
};

String formatDataPacket(uint8_t address,uint8_t *packet)
{
  String dataString = "";
  String format = "";

  // increment a byte to avoid packet type indicator
  uint8_t *packetPtr = packet + 1;
  DateTime dtData;
  char chData;
  uint8_t u8Data;
  uint16_t u16Data;
  float fData; 

  union typeSort theType;

  switch (address)
  {
    case DRONE_1_ADDRESS:
      format = dataFormat_drone_1;
      break;
    case DRONE_2_ADDRESS:
      format = dataFormat_drone_2;
      break;
  }
  if (format != "")
  {
    for (int i = 0; i < format.length(); i++)
    {
       switch (format.charAt(i))
      {
        case 't':
          dtData = dateTimeFromPacket(packetPtr);
          packetPtr += 15;
          dataString += dtData.timestamp();
          break;
        case 'c':
          dataString += (char) *packetPtr;
          packetPtr += 1;
          break;
        case 'u':
          u8Data = *(uint8_t *) packetPtr;
          packetPtr += 1;
          dataString += String(u8Data,DEC);
          break;
        case 'U':
          theType.bytes[0] = *packetPtr;
          theType.bytes[1] = *(packetPtr+1);
          u16Data = theType.uint16;
          packetPtr += 2;
          dataString += String(u16Data,DEC);
          break;
        case 'f': 
          theType.bytes[0] = *packetPtr;
          theType.bytes[1] = *(packetPtr+1);
          theType.bytes[2] = *(packetPtr+2);
          theType.bytes[3] = *(packetPtr+3);
          fData = theType.flt32 ;
          packetPtr += 4;
          dataString += String(fData,2);
          break;
      }
      if (i < format.length() - 1)
        dataString += ',';
    }
  }
  else
  {
    sendFormatStringRequest(address);
    for (int i = 0; i < 255; i++)
    {
      dataString += String((*(packetPtr+i)), HEX);
      dataString += ' ';
    }
  }
  return dataString;
}

void led(bool state)
{
  if (state)
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  else
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off 
}

bool boxTalking = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Firehawk II LoRa Receiver");
  Serial.println(receiverVersion);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa driver init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!manager.init())
    Serial.println("manager init failed");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setTxPower(20, false);
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(20, false);
  boxTalking = false;
}

int toDrone = 1;
String command = "";

void loop() {
  /*
  if (!boxTalking)
  {
    command = "";
    if (sendPacket(toDrone, PACKET_TYPE_PERMISSION_TO_TALK, (unit8_t *) command.c_str(),1))
    {
      boxTalking = true;
    }
    else
    {

    }
  }
  */
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int inchar = Serial.read();

    if ((inchar == '\r') || (inchar == '\n'))
    {
      if ((command == "1") || (command == "2"))
      {
        toDrone = command.toInt();
        Serial.print("Commands directed to drone ");
        Serial.println(command);
        command = "";
      }
      else
      {
        Serial.print("Sending command to drone ");
        Serial.print(toDrone,DEC);
        Serial.print(": ");
        Serial.println(command);
        commandRetry = MAX_COMMAND_RETRY;
        commandWaiting = true;
        //sendCommand((uint8_t)toDrone, command);
      }
    }
    else
    {
      command += (char) inchar;
    }
  }
    
    
  //Serial.println("Checking for radio message");
  if (manager.available()) {
    led(true);
    // Should be a message for us now
    uint8_t len = sizeof(buf);
    uint8_t from;
    String dataString;
    String formatString("");
    //Serial.println("About to receive");
    if (manager.recvfromAck(buf, &len, &from))
    {
      if (from == toDrone)
      {
        switch ((char) *buf)
        {
          case PACKET_TYPE_FORMAT:
            formatString += (char *) buf+1;
            setDataFormat(from, formatString);
            command = "";
            commandRetry = 0;
            break;
          case PACKET_TYPE_DATA:
            dataString = formatDataPacket(from, buf);
            Serial.print("Drone ");
            Serial.print(from,DEC);
            Serial.print("/* ");
            Serial.print(dataString);
            Serial.println(" */"); 
            break;
          case PACKET_TYPE_COMMAND_RESPONSE:
            Serial.print("Response from drone ");
            Serial.print(from,DEC);
            Serial.print(": ");
            Serial.println((char *) buf+1); 
            commandRetry = 0;
            command = "";
            break;
        }
      }
      led(false);

      // send command after radio message received from Drone
      if (commandWaiting && commandRetry)
      {
        int waittime = random(50,400);
        delay(waittime);
        led(true);
        sendCommand((uint8_t)toDrone, command);
        led(false);      
      }
      if (!commandRetry)
        command = "";

    } 
    else 
    {
      Serial.println("Receive failed");
    }
  }
}
