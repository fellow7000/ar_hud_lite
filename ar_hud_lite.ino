#include "Arduino.h"
#include <SPI.h>
#include "mcp2515_can.h"
#include <WiFiS3.h>
#include <WiFiUdp.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL SerialUSB
#else
    #define SERIAL Serial
#endif

#define DEBUG //Comment this line to switch off debug output once debugging is done. This will increase the performance of the SW
#define SERIAL_SPEED 2000000 //Serial speed for built-in USB Serial port
#define UDP_PORT 49155 //UDP port for Dynon data
#define UDP_PACKAGE_SIZE 400 //actually, the full package is 388 bytes long
#define DEFAULT_CAN_SPEED CAN_500KBPS //CAN speed for HUD
#define CAN_PIN 10 //CAN CS pin
#define DEV_ID 0x7e8 //CAN device ID for response to HUD
#define WIFI_SSID "Avionic_WiFi" //WiFi SSID of the Dynon device
#define WIFI_PASS "Avionic_Passwrod" //WiFi password of the Dynon device
#define HALT_APP while (1) {} //Halt the app

#define KT 0 //IAS in knots
#define KMH 1 //IAS in km/h
#define IAS KMH //IAS units

#define PSI 0 //Fuel pressure in PSI
#define BAR 1 //Fuel pressure in BAR
#define PSI_FACTOR 1 //Conversion factor for PSI to BAR
#define BAR_FACTOR 0.689476 //Conversion factor for BAR to PSI
#define FPS_OFFSET 40 //Offset for fuel pressure in PSI. Water temperature indicator of the HUD is used for fuel pressure. Needs to be adjusted to the real value
#define FUEL_PRES BAR //Fuel pressure units

#define I_RPM 0 //Engine RPM
#define I_AOA 1 //Angle of Attack
#define I_AOA_FACTOR 100 //Factor to convert AOA to the indicator value of RPM on the HUD
#define I_AOA_OFFSET 3500 //Offset to convert AOA to the indicator value  of RPM on the HUD
#define I_FPS I_AOA //Indicator units for the HUD
 
class Dynon {
private:
        /// @brief see Dynon Installation Manual for protocol details
        const static char code_ADAHRS = '1';
        const static char ver_ADAHRS = '1';
        const byte ias_pos = 23;
        const byte ias_lng = 4;
        const byte ias_dec_pos = 3;
        const byte aoa_pos = 43;
        const byte aoa_lng = 2;
        const byte aoa_dec_pos = -1;

        const static char code_SYSTEM = '2';
        const char ver_SYSTEM = '2';

        const static char code_EMS = '3';
        const char ver_EMS = '2';
        const byte rpmL_pos = 18;
        const byte rpmR_pos = 22;
        const byte rpm_lng = 4;
        const byte rpm_dec_pos = -1;

        const byte fuel_prs_pos = 35;
        const byte fuel_prs_lng = 3;
        const byte fuelPres_dec_pos = 2;

        const byte minStrLngth = 3; //minimum length of incoming string from Daynon to parse

        const char strStart = '!';
        const byte data_Type_Pos = 1;
        
        int status = WL_IDLE_STATUS; //WiFi status
        WiFiUDP udp;
        String ssid;
        String password;
        
        char packetBuffer[UDP_PACKAGE_SIZE]; //buffer to store incoming UDP packet data
        unsigned long lastPacketTime = 0;
        unsigned long packetTime = 0;
        long packetBufferSize = 0;

        String extractData(String& srcStr, int start_pos, int lng, int dec_pos) { //decPos - where to add a "." into the number
                   #ifdef DEBUG
                     Serial.println("String to extract a Substring");
                     Serial.println(srcStr);
                   #endif

                String subStr = "";
                for (int i = 0; i < lng; i++) {
                     if (i == dec_pos) subStr = subStr + '.';
                  subStr = subStr + (srcStr)[start_pos + i];
                }

                   #ifdef DEBUG
                   Serial.println("Extracted Substring");
                   Serial.println(subStr);
                   #endif

                return subStr;
        }
 
public:
  Dynon(String ssid, String psw, long UDP_Port = 0) : 
  ssid(ssid),
  password(psw),
  udpPort(UDP_Port) {};
 
  long udpPort = 0;
  byte ias_kt = 0;
  byte ias_kmh = 10;
  boolean iasOk = false;

  byte aoa_raw = 200;
  boolean aoaOk = false;
  int aoa_indicator = 0; //what will be displayed on the HUD

  word rpmL = 200;
  word rpmR = 200;
  boolean rpmOk = false;
  float fuelPrs_psi = 0.0;
  float fuelPrs_bar = 0.0;
  boolean fuelPrsOk = false;
 
  boolean dynonOk = false; //if communication to Dynon device is ok

  /// @brief Connect to Dynon device v. WiFi
  void connectToDynon() {
    if (WiFi.status() == WL_NO_MODULE) {
        SERIAL.println("No WiFi Module Detected");
        HALT_APP;
    }

    String fv = WiFi.firmwareVersion();

    SERIAL.println("On-board connectivity module firmware version:");
    SERIAL.println(fv.c_str());
    SERIAL.println("Latest connectivity module firmware version:");
    SERIAL.println(WIFI_FIRMWARE_LATEST_VERSION);

    int i = 0; //counter of attemps to connect
    int max_i = 3; //max attemps to connect

    while (status != WL_CONNECTED) {
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid.c_str(), password.c_str());

        if (status == WL_CONNECT_FAILED) {
          SERIAL.println("Connection to WiFi failed, retrying. Recheck connection settings");
          i++;
        }

        if (i >= max_i) {
          SERIAL.println("No further attemps to connect to WiFi. Recheck connection credentials. System halted");
          HALT_APP;
        }

        // wait 1 second for connection:
        delay(1000);
    }    
    
    SERIAL.println("Connected to WiFi");
    udp.begin(udpPort);
    SERIAL.println("UDP started on port " + String(udpPort));
  }

  /// @brief Get data from Dynon UDP
  /// @param dataStr 
  /// @param packageReadt 
  void getDataUDP(String& dataStr, bool& packetReady) {
    int packetSize = udp.parsePacket();

    if (packetSize) {
      packetTime = millis();
      #ifdef DEBUG
        SERIAL.println("UDP packet size: " + String(packetSize) +", delta-time: " + String(millis() - lastPacketTime) + " ms");
      #endif
      lastPacketTime = packetTime;
      
      dataStr.reserve(packetSize);
      udp.read(packetBuffer, packetSize);
      packetBuffer[packetSize] = 0;  // Null-terminate the string, not considering possible overflow
      dataStr = String(packetBuffer);
      packetReady = true;
      return;      
    } else {
        //LOG_DEBUG("No UDP Data");
        packetReady = false;
        return;
    }      
  }
 
  void parseDynonStr(String& sourceStr) {
    int start = 0;
    int end = sourceStr.indexOf("\n");

    #ifdef DEBUG
      Serial.println("Position of the first \\n :"+ String(end));
    #endif
    

    String substring = "";

    while (end != -1) {
        substring = sourceStr.substring(start, end);
        
        #ifdef DEBUG
          Serial.println("Substring to parse");
          Serial.println(substring);
        #endif

        assignValues(substring);
        start = end + 1; // Move start position past the "\r\n"
        end = sourceStr.indexOf("\n", start);
        #ifdef DEBUG
          Serial.println("Position of the next \\n :"+ String(end));
        #endif
    }

    // Check if there's any data left after the last "\r\n" and process it
    if (start <= sourceStr.length() - 1) {
        substring = sourceStr.substring(start, end);

        #ifdef DEBUG
          Serial.println("Substring to parse");
          Serial.println(substring);
        #endif

        assignValues(substring);
    }
  }

  void assignValues(String& sourceStr) {
    dynonOk = true;

        #ifdef DEBUG
          Serial.println("Dynon String for switch");
          Serial.println(sourceStr);
          Serial.println(sourceStr.length(), DEC);
        #endif
 
           if (sourceStr.length() < minStrLngth) {
             Serial.println("Incoming String is too short");
             dynonOk = false;
           }
 
           if (dynonOk && (sourceStr.length() >= 1) && (sourceStr[0] != strStart)) {
             Serial.println("! not found found on position 0");
             dynonOk = false;
           }
 
           if (dynonOk && (sourceStr.length() >= 2) && ((sourceStr[1] < '1') || (sourceStr[1] > '9'))) {
             Serial.println("Provided data version is not a number");
             dynonOk = false;
           }
 
           if (!dynonOk) {
             iasOk = false;
             rpmOk = false;
             fuelPrsOk = false;
             return;
           }
 
           switch (sourceStr[data_Type_Pos]) {
             case code_ADAHRS :
              parseADAHRS(sourceStr);
              #ifdef DEBUG
                Serial.println(ias_kt, DEC);
              #endif
              break;
             case code_SYSTEM : //currently we do not need system parsing and can switch this stream off
              #ifdef DEBUG
                Serial.println("System String Detected");
              #endif
              break;
             case code_EMS :
              parseEMS(sourceStr);
              break;
             default:
              #ifdef DEBUG
               Serial.println("Not supported source");
              #endif
              break;
           }
  }
 
  void parseADAHRS(String adahrsStr) {
 
       #ifdef DEBUG
         Serial.println("ADAHRS String to parse");
         Serial.println(adahrsStr);
         Serial.println(adahrsStr.length(), DEC);
       #endif
 
     if (adahrsStr.length() < (ias_pos + ias_lng - 1)) {
      Serial.println("Incoming String is too short to decode IAS");
      dynonOk = false;
      iasOk = false;
      return;
    }
 
    String valueStr = extractData(adahrsStr, ias_pos, ias_lng, ias_dec_pos);
 
    ias_kt = round(valueStr.toFloat());


    if ((ias_kt * 1.852) > 255) { //need to limit spd_kmh output as we have only one byte to store the value
      ias_kmh = 255;
    } else {
       ias_kmh = round(ias_kt * 1.852);
    }
 
    #ifdef DEBUG
      Serial.println("IAS");
      Serial.println(valueStr);
      Serial.println(ias_kt, DEC);
      Serial.println(ias_kmh, DEC);
    #endif

    valueStr = extractData(adahrsStr, aoa_pos, aoa_lng, aoa_dec_pos);
 
    aoa_raw = round(valueStr.toFloat());
    aoa_indicator = aoa_raw * I_AOA_FACTOR - I_AOA_OFFSET;
 
    #ifdef DEBUG
      Serial.println("AOA");
      Serial.println(valueStr);
      Serial.println(aoa_raw, DEC);
      Serial.println(aoa_indicator, DEC);
    #endif    
 
  }
 
  void parseEMS(String& emsStr) {
    
    #ifdef DEBUG
      Serial.println("EMS String to parse");
      Serial.println(emsStr);
      Serial.println((emsStr).length(), DEC);
    #endif
 
    if ((emsStr).length() < (fuel_prs_pos + fuel_prs_lng - 1)) {
      Serial.println("Incoming String is too short to decode RPM and Fuel Pressure");
      dynonOk = false;
      rpmOk = false;
      fuelPrsOk = false;
      return;
    }
 
     String valueStr = extractData(emsStr, rpmL_pos, rpm_lng, rpm_dec_pos);
     rpmL = valueStr.toInt();
             #ifdef DEBUG
      Serial.println("Left RPM");
      Serial.println(valueStr);
      Serial.println(rpmL, DEC);
             #endif
 
     valueStr = extractData(emsStr, rpmR_pos, rpm_lng, rpm_dec_pos);
     rpmR = valueStr.toInt();
    #ifdef DEBUG
      SERIAL.println("Right RPM");
      SERIAL.println(valueStr);
      SERIAL.println(rpmR, DEC);
    #endif
 
     valueStr = extractData(emsStr, fuel_prs_pos, fuel_prs_lng, fuelPres_dec_pos);
     fuelPrs_psi = valueStr.toFloat() * PSI_FACTOR + FPS_OFFSET;
     fuelPrs_bar = valueStr.toFloat() * BAR_FACTOR + FPS_OFFSET;
    #ifdef DEBUG
      SERIAL.println("Fuel Pressure string");
      SERIAL.println(valueStr);
      SERIAL.println("Fuel Pressure PSI");
      SERIAL.println(fuelPrs_psi, DEC);
      SERIAL.println("Fuel Pressure BAR");
      SERIAL.println(fuelPrs_bar, DEC);
    #endif
 
  }
 
};
 
const int SPI_CS_PIN = CAN_PIN;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
 
word devID = DEV_ID;
 
byte obd_req_byte1 = 0x2;
byte obd_req_byte2 = 0x1;
 
const byte pid0x0 = 0x0;
const byte pid0x5_Temp = 0x5;
const byte pid0xC_RPM = 0xC;
const byte pid0xD_Speed = 0xD;
const byte pid0x20 = 0x20;
const byte pid0x40 = 0x40;
 
const byte prefx03 = 0x03;
const byte prefx04 = 0x04;
const byte prefx06 = 0x06;
const byte prefx10 = 0x10;
const byte prefx20 = 0x20;
const byte prefx41 = 0x41;
const byte sufx0 = 0x0;
const byte sufxAA = 0xAA;
 
byte len = 0; // data length of the incoming CAN message
byte buf[8]; //CAN payload

byte can_msg_out[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //outbound CAN message
byte can_out_lngth = 8; //length of the payload
 
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
 
Dynon myDyn(WIFI_SSID, WIFI_PASS, UDP_PORT); //create an instance of the Dynon class
 
//The setup function is called once at startup of the sketch
void setup()
{
    SERIAL.begin(SERIAL_SPEED);

    myDyn.connectToDynon(); //connecto to Avionics v. WiFi

    // reserve 240 bytes for the inputString:
    inputString.reserve(UDP_PACKAGE_SIZE);
    while(!Serial){};
 
    while (CAN_OK != CAN.begin(DEFAULT_CAN_SPEED)) {             // init can bus : baudrate = 500k
              SERIAL.println("CAN BUS Shield init fail, trying again...");
        delay(100);
    }

    SERIAL.println("CAN BUS Shield Init Ok!");
}
 
// The loop function is called in an endless loop
void loop()
{
    //Try to get data from the Dynon v. UDP

    //check if we have an input string
    myDyn.getDataUDP(inputString, stringComplete);

    if (stringComplete) {
      #ifdef DEBUG
        Serial.println(inputString);
      #endif
      // clear the string:
      myDyn.parseDynonStr(inputString);
      inputString = "";
      stringComplete = false;
    }
 
    //check if we have an input CAN message from the HUD
    if (CAN_MSGAVAIL == CAN.checkReceive()) {         // check if data coming
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
 
        unsigned long canId = CAN.getCanId();
 
          #ifdef DEBUG
          Serial.println("-----------------------------");
          Serial.print("Get data from ID: 0x");
          Serial.println(canId, HEX);             
 
          for (int i = 0; i < len; i++) { // print the data
            Serial.print(buf[i], HEX);
            Serial.print("\t");
          }
          Serial.println();
        #endif

        process_CAN_Req(len, buf);
    }
}

/// @brief Process CAN requests from the HUD
/// @param len 
/// @param buf 
void process_CAN_Req(byte len, byte buf[8]) {
       if (len<3) {
             #ifdef DEBUG
             Serial.println("CAN Data Length is less than 3 bytes");
             #endif
             return;
       }
 
       if ((buf[0] != obd_req_byte1) && (buf[1] != obd_req_byte2)) {
             #ifdef DEBUG
             Serial.println("Unknown OBD-2 Request");
             #endif
       }
 
       switch (buf[2]) {
             case pid0x0: gen_PID0_Response(can_msg_out);
                    break;
             case pid0x5_Temp: gen_Temp_Response(FUEL_PRES == PSI ? myDyn.fuelPrs_psi : myDyn.fuelPrs_bar);
                    break;
             case pid0xC_RPM: gen_RPM_Response(I_FPS == I_RPM ? myDyn.rpmL : myDyn.aoa_indicator);
                    break;
             case pid0xD_Speed: gen_Speed_Response(IAS == KMH ? myDyn.ias_kmh : myDyn.ias_kt);
                    break;
             case pid0x20: gen_PID20_40_Response(pid0x20);
                    break;
             case pid0x40: gen_PID20_40_Response(pid0x40);
                    break;
             default:
                    #ifdef DEBUG
                    Serial.println("Unsupported PID Request");
                    #endif
                    return;
       }
 
       CAN.sendMsgBuf(devID, 0, can_out_lngth, can_msg_out);
 
}

/// @brief Techincal response to the HUD.
/// @param buf_out
void gen_PID0_Response(byte *buf_out) {
       buf_out[0] = prefx06;
       buf_out[1] = prefx41;
       buf_out[2] = pid0x0;
       buf_out[3] = 0b10001000;
       buf_out[4] = 0b11000;
       buf_out[5] = 0b0;
       buf_out[6] = 0b10000;
       buf_out[7] = 0b0;
 
       //out_lng = 8;
       can_out_lngth = 8;
       #ifdef DEBUG
       Serial.println("Sending PID 0x0 (Supported PID) Response");
       #endif
}
 
/// @brief Techincal response to the HUD.
/// @param pidNr 
void gen_PID20_40_Response(byte pidNr) {
       can_msg_out[0] = prefx06;
       can_msg_out[1] = prefx41;
       can_msg_out[2] = pidNr;
       can_msg_out[3] = sufx0;
       can_msg_out[4] = sufx0;
       can_msg_out[5] = sufx0;
       can_msg_out[6] = sufx0;
       can_msg_out[7] = sufx0;
       can_out_lngth = 8;
 
       #ifdef DEBUG
       if (pidNr == pid0x20)
       Serial.println("Sending PID 0x20 (Supported PID) Response");
       else if (pidNr == pid0x40) {
             Serial.println("Sending PID 0x40 (Supported PID) Response");
       } else {
             Serial.println("Unsupported PID request");
       }
       #endif
}
 
/// @brief Water Temperature Response. We use it to display Fuel Pressure
/// @param tempRaw 
void gen_Temp_Response(byte tempRaw) {
       can_msg_out[0] = prefx03;
       can_msg_out[1] = prefx41;
       can_msg_out[2] = pid0x5_Temp;
       can_msg_out[3] = tempRaw;
       can_out_lngth = 4;
 
       #ifdef DEBUG
       Serial.println("Sending PID 0x5 (Temp) Response");
       #endif
}
 
/// @brief Speed Response. We use it to display IAS
/// @param speedRaw 
void gen_Speed_Response(byte speedRaw) {
       can_msg_out[0] = prefx03;
       can_msg_out[1] = prefx41;
       can_msg_out[2] = pid0xD_Speed;
       can_msg_out[3] = speedRaw;
       can_out_lngth = 4;
 
       #ifdef DEBUG
       Serial.println("Sending PID 0xD (Speed) Response");
       #endif
}
 
/// @brief RPM Response. We use it to display RPM or AOA depending on the HUD settings
/// @param rpmRaw 
void gen_RPM_Response(word rpmRaw) {
       byte rpm_byte1 = 0; //first byte of RPM encoding for CAN message
       byte rpm_byte2 = 0; //second byte of RPM encoding for CAN message
       byte rpm1_Adder = 0;
 
       word speed4 = (rpmRaw * 4);
       word rpmDec = (speed4 % 100);
       word rpmThnds = (speed4 - rpmDec);
       byte rpm1 = int((rpmThnds / 256));
       word rpm2_raw = (rpmDec + (rpmThnds - (rpm1 * 256)));
 
       if (rpm2_raw >= 256) {
             rpm1_Adder = (rpm2_raw / 256);
             rpm_byte2 = (rpm2_raw - 256);
       } else {
             rpm_byte2 = rpm2_raw;
       }
 
       rpm_byte1 = (rpm1 + rpm1_Adder);
 
       #ifdef DEBUG
       word rpm_ver = (((rpm_byte1 * 256) + rpm_byte2) / 4); //verify RPM calc
       Serial.print("Verified RPM: ");
       Serial.println(rpm_ver, DEC);
       #endif
 
       can_msg_out[0] = prefx03;
       can_msg_out[1] = prefx41;
       can_msg_out[2] = pid0xC_RPM;
       can_msg_out[3] = rpm_byte1;
       can_msg_out[4] = rpm_byte2;
       can_out_lngth = 5;
 
       #ifdef DEBUG
       Serial.println("Sending PID 0xC (RPM) Response");
       #endif
}