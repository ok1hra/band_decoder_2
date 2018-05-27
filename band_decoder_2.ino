#include <Arduino.h>

/*

  Band decoder MK2 with TRX control output for Arduino rev.0.4
-----------------------------------------------------------
  https://remoteqth.com/wiki/index.php?page=Band+decoder+MK2
  2018-05 by OK1HRA

  ___               _        ___ _____ _  _
 | _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
 |   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
 |_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Features:
Support inputs
  * PTT detector - if PTT on, outputs not change
  * SERIAL terminal (ASCII)
  * ICOM CI-V
  * KENWOOD - CAT
  * YAESU BCD
  * ICOM ACC voltage
  * YAESU CAT - TRX since 2008 ascii format
  * YAESU CAT old binary format (tested on FT-817)

Outputs
  * 14 local relay
  * 14 remote relay
  * Yaesu BCD
  * Serial echo
  * Icom CIV
  * Kenwood CAT
  * YAESU CAT - TRX since 2008 ascii format

  Major changes
  -------------
  - LCD support
  - Icom with request mode
  - PTT input block changes during transmit
  - own board with all smd parts (without arduino nano module)
  - without relays, only control driver outputs
  - optional ethernet module

  Changelog
  ---------
  2018-05 mk2 initial release

*/
//=====[ Hardware ]=============================================================================================

// #define EthModule          // enable Ethernet module if installed
// #define __USE_DHCP__       // Uncoment to Enable DHCP
#define LCD                   // Uncoment to Enable I2C LCD
#define LcdI2Caddress  0x3F   // 0x27 0x3F - may be find with I2C scanner https://playground.arduino.cc/Main/I2cScanner

//=====[ Inputs ]=============================================================================================

// #define YAESU_BCD          // TTL BCD in A
// #define ICOM_ACC           // voltage 0-8V on pin4 ACC(2) connector - need calibrate table
// #define INPUT_SERIAL       // telnet ascii input - cvs format [band],[freq]\n
#define ICOM_CIV           // read frequency from CIV

// #define KENWOOD_PC         // RS232 CAT
// #define YAESU_CAT          // RS232 CAT YAESU CAT since 2015 ascii format
// #define YAESU_CAT_OLD      // Old binary format RS232 CAT ** tested on FT-817 **

//=====[ Outputs ]============================================================================================

// #define REMOTE_RELAY       // TCP/IP remote relay - need install and configure TCP232 module
// #define SERIAL_echo        // Feedback on serial line in same baudrate, CVS format <[band],[freq]>\n
// #define ICOM_CIV_OUT       // send frequency to CIV ** you must set TRX CIV_ADRESS, and disable ICOM_CIV **
// #define KENWOOD_PC_OUT     // send frequency to RS232 CAT ** for operation must disable REQUEST **
// #define YAESU_CAT_OUT      // send frequency to RS232 CAT ** for operation must disable REQUEST **

//=====[ Settings ]===========================================================================================

#define SERBAUD        9600   // [baud] Serial port in/out baudrate
#define WATCHDOG       10     // [sec] determines the time, after which the all relay OFF, if missed next input data - uncomment for the enabled
#define REQUEST        500    // [ms] use TXD output for sending frequency request
#define CIV_ADRESS   0x56     // CIV input HEX Icom adress (0x is prefix)
#define CIV_ADR_OUT  0x56     // CIV output HEX Icom adress (0x is prefix)
// #define ENABLE_DIVIDER     // for highest voltage D-SUB pin 13 inputs up to 24V - need short JP9

//=====[ FREQUEN RULES ]===========================================================================================

const long Freq2Band[16][2] = {/*
Freq Hz from       to   Band number
*/   {1810000,   2000000},  // #1 [160m]
     {3500000,   3800000},  // #2  [80m]
     {7000000,   7200000},  // #3  [40m]
    {10100000,  10150000},  // #4  [30m]
    {14000000,  14350000},  // #5  [20m]
    {18068000,  18168000},  // #6  [17m]
    {21000000,  21450000},  // #7  [15m]
    {24890000,  24990000},  // #8  [12m]
    {28000000,  29700000},  // #9  [10m]
    {50000000,  52000000},  // #10  [6m]
   {144000000, 146000000},  // #11  [2m]
   {430000000, 440000000},  // #12  [70cm]
   {1240000000, 1300000000},  // #13  [23cm]
   {2300000000, 2450000000},  // #14  [13cm]
   {3300000000, 3500000000},  // #15  [9cm]
   {5650000000, 5850000000},  // #16  [6cm]
};

//=====[ Sets band -->  to output in MATRIX table ]===========================================================

        const boolean matrix[17][16] = { /*

        Band 0 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  1 }, /* first eight shift register board
\       Band 1 --> */ { 1,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
 \      Band 2 --> */ { 0,  1,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
  \     Band 3 --> */ { 0,  0,  1,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
   \    Band 4 --> */ { 0,  0,  0,  1,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
    \   Band 5 --> */ { 0,  0,  0,  0,  1,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
     \  Band 6 --> */ { 0,  0,  0,  0,  0,  1,  0,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
IN    ) Band 7 --> */ { 0,  0,  0,  0,  0,  0,  1,  0,    0,  0,  0,  0,  0,  0,  0,  0 }, /*
     /  Band 8 --> */ { 0,  0,  0,  0,  0,  0,  0,  1,    0,  0,  0,  0,  0,  0,  0,  0 }, /*

    /   Band 9 --> */ { 0,  0,  0,  0,  0,  0,  0,  0,    1,  0,  0,  0,  0,  0,  0,  0 }, /* second eight shift register board
   /    Band 10 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  1,  0,  0,  0,  0,  0,  0 }, /* (optional)
  /     Band 11 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  1,  0,  0,  0,  0,  0 }, /*
 /      Band 12 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  1,  0,  0,  0,  0 }, /*
/       Band 13 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  1,  0,  0,  0 }, /*
        Band 14 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  1,  0,  0 }, /*
        Band 15 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  1,  0 }, /*
        Band 16 -> */ { 0,  0,  0,  0,  0,  0,  0,  0,    0,  0,  0,  0,  0,  0,  0,  1 }, /*
                        |   |   |   |   |   |   |   |     |   |   |   |   |   |   |   |
                        V   V   V   V   V   V   V   V     V   V   V   V   V   V   V   V
                     ----------------------------------  ---------------------------------
                     |  1   2   3   4   5   6   7   8     9  10  11  12  13  14  15  16  |
                     ----------------------------------  ---------------------------------
                                                   OUTPUTS
                                    (for second eight need aditional board)*/
        };

//=====[ BCD OUT ]===========================================================================================

        const boolean BCDmatrixOUT[4][16] = { /*
        --------------------------------------------------------------------
        Band # to output relay   0   1   2   3   4   5   6   7   8   9  10
        (Yaesu BCD)                 160 80  40  30  20  17  15  12  10  6m
        --------------------------------------------------------------------
                                 |   |   |   |   |   |   |   |   |   |   |
                                 V   V   V   V   V   V   V   V   V   V   V
                            */ { 0,  1,  0,  1,  0,  1,  0,  1,  0,  1,  0, 1, 0, 1, 0, 1 }, /* --> DB25 Pin 11
                            */ { 0,  0,  1,  1,  0,  0,  1,  1,  0,  0,  1, 1, 0, 0, 1, 1 }, /* --> DB25 Pin 24
                            */ { 0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0, 0, 1, 1, 1, 1 }, /* --> DB25 Pin 12
                            */ { 0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1, 1, 1, 1, 1, 1 }, /* --> DB25 Pin 25
        */};

//============================================================================================================

#if defined(LCD)
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(LcdI2Caddress,16,2);
  long LcdRefresh[2]{0,500};
  const char* ANTname[17] = {
      "Out of band",  // Band 0 (no data)
      "Dipole",       // Band 1
      "Vertical",     // Band 2
      "3el Yagi",     // Band 3
      "Windom",       // Band 4
      "DeltaLoop",    // Band 5
      "20m Stack",    // Band 6
      "DeltaLoop",    // Band 7
      "HB9",          // Band 8
      "Dipole",       // Band 9
      "5el Yagi",     // Band 10
      "7el Yagi",     // Band 11
      "24el",         // Band 12
      "20el quad",    // Band 13
      "Dish 1.2m",    // Band 14
      "Dish 1.2m",    // Band 15
      "Dish 1m",      // Band 16
  };
  // byte LockChar[8] = {0b00100, 0b01010, 0b01010, 0b11111, 0b11011, 0b11011, 0b11111, 0b00000};
  uint8_t LockChar[8] = {0x4,0xa,0xa,0x1f,0x1b,0x1b,0x1f,0x0};
  bool LcdNeedRefresh = false;
#endif

#if defined(EthModule)
  //  #include <util.h>
  #include <Ethernet2.h>
  #include <Dhcp.h>
  #include <EthernetServer.h>
  #include <SPI.h>
  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE};
  IPAddress ip(192, 168, 1, 220);         // IP
  IPAddress gateway(192, 168, 1, 200);    // GATE
  IPAddress subnet(255, 255, 255, 0);     // MASK
  IPAddress myDns(8, 8, 8, 8);            // DNS (google pub)
  EthernetServer server(80);              // server PORT
  String HTTP_req;
  byte BOARD_ID;

  unsigned int UdpCommandPort = 88;       // local UDP port listen to command
  #define UDP_TX_PACKET_MAX_SIZE 40       // MIN 30
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
  int UDPpacketSize;
  EthernetUDP UdpCommand; // An EthernetUDP instance to let us send and receive packets over UDP
  IPAddress BroadcastIP(0, 0, 0, 0);        // Broadcast IP address
  int BroadcastPort       = 88;             // destination broadcast packet port
  IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
  int RemoteSwPort         = 0;             // remote UDP IP switch port
  byte DetectedRemoteSw[16][5];             // detect by RX broadcast packet - storage by ID (ID=rows)
  int BandDecoderChange    = 0;             // If band change, send query packet to Remote IP switch
  long RemoteSwLatency[2];                  // start, stop mark
  byte RemoteSwLatencyAnsw = 1;             // answer (offline) detect
  byte TxUdpBuffer[10];
  long IpTimeout[1][2] = {0, 60000};          // UDP Broadcast packet [8][0-timer/1-timeout]

#endif

// PINOUTS
const int VoltagePin = A0;        // measure input voltage
  const int EncBPin = A0;         // Encoder B
const int Id1Pin = A1;            // ID switch
const int BcdIn1Pin = A2;         // BCD-1 in/out
  const int ShiftInClockPin = A2; // [ShiftIn]
const int ADPin = A3;             // [BD/ROT]
  const int SequencerPin = A3;    // [IPsw/BD]
// const int SdaPin = A4;         // [LCD]
  const int Id4Pin = A4;          //
// const int SclPin = A5;         // [LCD]
const int Id2Pin = A6;            //
const int Id3Pin = A7;            //
const int PttDetectorPin = 2;     // PTT in - [interrupt]
const int BcdIn4Pin = 3;          // BCD-4 in/out
  const int EncAShiftInPin = 3;   // Encoder+Keyboard - [interrupt]
const int BcdIn3Pin = 4;          // BCD-3 in/out
  const int ShiftInDataPin = 4;   // [ShiftIn]
const int BcdIn2Pin = 5;          // BCD-2 in/out
  const int ShiftInLatchPin = 5;  // [ShiftIn]
const int PttOffPin = 6;          // PTT out OFF switch
const int ShiftOutDataPin = 7;    // DATA
const int ShiftOutLatchPin = 8;   // LATCH
const int ShiftOutClockPin = 9;   // CLOCK

int BAND = 0;
int previousBAND = -1;
long freq = 0;
bool PTT = false;
long PttTiming[2]={0, 10};            // debouncing time and also minimal PTT on time in ms
float DCinVoltage;
float ResistorCoeficient = 6.0;
long VoltageRefresh[2] = {0, 3000};   // refresh in ms
float ArefVoltage = 4.303;            // Measure on Aref pin 20 for calibrate
float Divider = 1;

int NumberOfBoards = 1;    // number of eight byte shift register 0-x
byte ShiftByte[5];

// int SelectOut = 0;
// int x;
long RequestTimeout[2]={0, REQUEST};
int watchdog2 = 500;     // REQUEST refresh time [ms]
int previous2;
int timeout2;

#if defined(WATCHDOG)
    int previous;
    int timeout;
    long WatchdogTimeout[2] = {-WATCHDOG*1000, WATCHDOG*1000};
#endif
#if defined(ICOM_ACC)
    int VALUE = 0;
    int prevVALUE=0;
    float VOLTAGE = 0;
    int band = 0;
    int counter = 0;
#endif
#if defined(YAESU_BCD)
    long BcdInRefresh[2] = {0, 1000};   // refresh in ms
#endif
#if defined(REMOTE_RELAY)
    int watchdog3 = 1000;     // send command to relay refresh time [ms]
    int previous3;
    int timeout3;
#endif
#if defined(KENWOOD_PC) || defined(YAESU_CAT)
    int lf = 59;  // 59 = ;
#endif
#if defined(KENWOOD_PC)
    char rdK[37];   //read data kenwood
    String rdKS;    //read data kenwood string
#endif
#if defined(YAESU_CAT)
    char rdY[37];   //read data yaesu
    String rdYS;    //read data yaesu string
#endif
#if defined(YAESU_CAT_OLD)
    byte rdYO[37];   //read data yaesu
    String rdYOS;    //read data yaesu string
#endif
#if defined(ICOM_CIV) || defined(ICOM_CIV_OUT)
    int fromAdress = 0xE0;              // 0E
    byte rdI[10];   //read data icom
    String rdIS;    //read data icom string
    long freqPrev1;
    byte incomingByte = 0;
    int state = 1;  // state machine
#endif
#if defined(KENWOOD_PC_OUT) || defined(YAESU_CAT_OUT)
    long freqPrev2;
#endif
#if defined(BCD_OUT)
    char BCDout;
#endif

//---------------------------------------------------------------------------------------------------------

void setup() {
  #if defined(YAESU_CAT_OLD)
    Serial.begin(SERBAUD, SERIAL_8N2);
    Serial.setTimeout(10);
  #else
    Serial.begin(SERBAUD);
    Serial.setTimeout(10);
  #endif

  #if defined(KENWOOD_PC) || defined(YAESU_CAT)
    // CATdata.reserve(200);          // reserve bytes for the CATdata
  #endif

  pinMode(VoltagePin, INPUT);

  #if defined(ICOM_ACC)
    pinMode(ADPin, INPUT);
  #else
    pinMode(SequencerPin, OUTPUT);
  #endif

  #if defined(EthModule)
    pinMode(Id1Pin, INPUT);
      digitalWrite(Id1Pin, INPUT_PULLUP);
    pinMode(Id2Pin, INPUT);
      digitalWrite(Id2Pin, INPUT_PULLUP);
    pinMode(Id3Pin, INPUT);
      digitalWrite(Id3Pin, INPUT_PULLUP);
    GetBoardId();
  #endif

  pinMode(PttDetectorPin, INPUT);
    digitalWrite(PttDetectorPin, HIGH);

  pinMode(PttOffPin, OUTPUT);
  #if defined(YAESU_BCD)
    pinMode(BcdIn1Pin, INPUT);
      // digitalWrite(Id3Pin, INPUT_PULLUP);
      digitalWrite(BcdIn1Pin, HIGH);
    pinMode(BcdIn2Pin, INPUT);
      digitalWrite(BcdIn2Pin, HIGH);
    pinMode(BcdIn3Pin, INPUT);
      digitalWrite(BcdIn3Pin, HIGH);
    pinMode(BcdIn4Pin, INPUT);
      digitalWrite(BcdIn4Pin, HIGH);
  #else
    pinMode(BcdIn1Pin, OUTPUT);
    pinMode(BcdIn2Pin, OUTPUT);
    pinMode(BcdIn3Pin, OUTPUT);
    pinMode(BcdIn4Pin, OUTPUT);
  #endif
  pinMode(ShiftOutDataPin, OUTPUT);
  pinMode(ShiftOutLatchPin, OUTPUT);
  pinMode(ShiftOutClockPin, OUTPUT);

  analogReference(EXTERNAL);

  #if defined(LCD)
    lcd.backlight();
    lcd.init();
    lcd.createChar(0, LockChar);
    lcd.clear();
    lcd.setCursor(0,0);
      #if defined(INPUT_SERIAL)
        lcd.print("SERIAL      ");
      #endif
      #if defined(ICOM_CIV)
        lcd.print("ICOM        ");
        lcd.print(CIV_ADRESS, HEX);
        lcd.print("h");
      #endif
      #if defined(ICOM_ACC)
      lcd.print("ICOM ACC    ");
      #endif
      #if defined(KENWOOD_PC)
        lcd.print("KENWOOD      ");
      #endif
      #if defined(YAESU_CAT) || defined(YAESU_CAT_OLD)
        lcd.print("YAESU       ");
      #endif
      #if defined(YAESU_BCD)
        lcd.print("YAESU BCD   ");
      #endif
      #if !defined(YAESU_BCD) && !defined(ICOM_ACC)
        lcd.setCursor(10,0);
        lcd.print(SERBAUD);
      #endif
    lcd.setCursor(0,1);
    lcd.print("DC input ");
    lcd.print(volt(analogRead(VoltagePin), ResistorCoeficient));
    lcd.print("V");
    delay(3000);
    lcd.clear();
  #endif

  #if defined(EthModule)
    #if defined __USE_DHCP__
      Ethernet.begin(mac);
    #else
    Ethernet.begin(mac, ip, myDns, gateway, subnet);
    #endif
    server.begin();
    #if defined(SERIAL_echo)
      Serial.print(F("Web server is at http://"));
      Serial.println(Ethernet.localIP());
    #endif
    #if defined(LCD)
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("IP address:"));
      lcd.setCursor(1, 1);
      lcd.print(Ethernet.localIP());
      delay(2500);
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("Board Net ID:"));
      lcd.print(BOARD_ID);
      delay(2500);
    #endif

    // SendBroadcastUdp();

  #endif
  // ANTname[0] = " [timeout]  ";
  InterruptON(1,1); // ptt, enc
}
//---------------------------------------------------------------------------------------------------------

void loop() {
  // EthernetServer();
  BandDecoderInput();
  BandDecoderOutput();
  LcdDisplay();
  watchDog();
  FrequencyRequest();
  PttOff();
  IncomingUDP();
}

// SUBROUTINES ---------------------------------------------------------------------------------------------------------

void IncomingUDP(){
  #if defined(EthModule)
    InterruptON(0,0); // ptt, enc
      // COMMANDS [port 88]
      UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
      if (UDPpacketSize){
        UdpCommand.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);      // read the packet into packetBufffer


        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Switch BROADCAST - STORAGE IP by received ID in 'DetectedRemoteSw' array (rows = Switch ID)
        if (packetBuffer[0] == 'b' && packetBuffer[1] == ':' && packetBuffer[2] == 's' && packetBuffer[4] == ';'){
          IPAddress TmpAddr = UdpCommand.remoteIP();
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [0]=TmpAddr[0];     // Switch IP addres storage to array
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [1]=TmpAddr[1];     // (int)packetBuffer[3] - 48 = convert char number to DEC
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [2]=TmpAddr[2];
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [3]=TmpAddr[3];
          // DetectedRemoteSw [(int)packetBuffer[3] - 48] [4]=UdpCommand.remotePort();
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[3])] [0]=TmpAddr[0];     // Switch IP addres storage to array
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[3])] [1]=TmpAddr[1];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[3])] [2]=TmpAddr[2];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[3])] [3]=TmpAddr[3];
          DetectedRemoteSw [hexToDecBy4bit(packetBuffer[3])] [4]=UdpCommand.remotePort();
          RemoteSwLatencyAnsw = 1;           // answer packet received

          // Serial2.println();
          // Serial2.print("RX b:s");
          // Serial2.print(packetBuffer[3]);
          // Serial2.println(";");
          // Serial2.println(hexToDecBy4bit(packetBuffer[3]), HEX);
          //
          // Serial2.print(packetBuffer[3], DEC);
          // Serial2.println(" ");
          // Serial2.println((int)packetBuffer[3] - 48, DEC);
          //
          // for (i = 0; i < 16; i++) {
          //   Serial2.print(i);
          //   Serial2.print("  ");
          //   Serial2.print(DetectedRemoteSw [i] [0]);
          //   Serial2.print(".");
          //   Serial2.print(DetectedRemoteSw [i] [1]);
          //   Serial2.print(".");
          //   Serial2.print(DetectedRemoteSw [i] [2]);
          //   Serial2.print(".");
          //   Serial2.print(DetectedRemoteSw [i] [3]);
          //   Serial2.print(":");
          //   Serial2.println(DetectedRemoteSw [i] [4]);
          // }
        }

        //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Switch ANSWER > SET LED - Bank0-2
        if (packetBuffer[0] == 's' && packetBuffer[1] == ':'){
          RemoteSwLatency[1] = (millis()-RemoteSwLatency[0])/2; // set latency (half path in ms us/2/1000)
          RemoteSwLatencyAnsw = 1;           // answer packet received

          // if (ACC_KEYBOARD==1 && KeyboardAnswLed==1 && HowRemoteSwitchID()<7){ // and ID < 7 (bank A/B)
            // Serial2.print("Remote IP switch ID: ");
            // Serial2.println(HowRemoteSwitchID());

            // need if RX answer from band change query
            // rxShiftInButton[0] = packetBuffer[2];
            // rxShiftInButton[1] = packetBuffer[3];
            // rxShiftInButton[2] = packetBuffer[4];

            // digitalWrite(ShiftOutLatchPin, LOW);  // ready for receive data
            // // shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, packetBuffer[4]);    // bankC
            // shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, packetBuffer[3]);    // bankB
            // shiftOut(ShiftOutDataPin, ShiftOutClockPin, MSBFIRST, packetBuffer[2]);    // bankA
            // digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin
          // }
        }

        memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer
      }

    InterruptON(1,1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------

void SendBroadcastUdp(){
  #if defined(EthModule)
  InterruptON(0,0); // ptt, enc
  Serial.print("TX Broadcast ");
  BroadcastIP = ~Ethernet.subnetMask() | Ethernet.gatewayIP();
  Serial.print(BroadcastIP);
  Serial.print(":");
  Serial.print(BroadcastPort);

  UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
  // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
    UdpCommand.print("b:o");
    UdpCommand.write(BOARD_ID);
    UdpCommand.print(";");
  UdpCommand.endPacket();
  Serial.print("   ms ");
  Serial.println(IpTimeout[0][0]);

  IpTimeout[0][0] = millis();                      // set time mark
  InterruptON(1,1); // ptt, enc
  #endif
}
//-------------------------------------------------------------------------------------------------------

unsigned char hexToDecBy4bit(unsigned char hex)
// convert a character representation of a hexidecimal digit into the actual hexidecimal value
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}
//-------------------------------------------------------------------------------------------------------

void InterruptON(int ptt, int enc){
  if(ptt==0){
    detachInterrupt(digitalPinToInterrupt(PttDetectorPin));
  }else if(ptt==1){
    attachInterrupt(digitalPinToInterrupt(PttDetectorPin), PttDetector, FALLING);  // need detachInterrupt in IncomingUDP() subroutine
  }
}
//---------------------------------------------------------------------------------------------------------

void PttDetector(){   // call from interupt
  digitalWrite(PttOffPin, HIGH);
  PTT = true;
  LcdNeedRefresh = true;
  PttTiming[0]=millis();
}
//---------------------------------------------------------------------------------------------------------

void PttOff(){
  if(PTT==true && millis()-PttTiming[0] > PttTiming[1] && digitalRead(PttDetectorPin)==HIGH && BAND!=0){
    digitalWrite(PttOffPin, LOW);
    PTT = false;
    LcdNeedRefresh = true;
    // bandSET();               //------------????????
  }
}

//---------------------------------------------------------------------------------------------------------

void FrequencyRequest(){
  if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){

    #if defined(ICOM_CIV)
      txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
    #endif

    #if defined(KENWOOD_PC)
          Serial.print("IF;");
          Serial.flush();       // Waits for the transmission of outgoing serial data to complete
    #endif

    RequestTimeout[0]=millis();
  }
}
//---------------------------------------------------------------------------------------------------------

void Space(int MAX, int LENGHT, char CHARACTER){
  int NumberOfSpace = MAX-LENGHT;
  if(NumberOfSpace>0){
    for (int i=0; i<NumberOfSpace; i++){
      lcd.print(CHARACTER);
    }
  }
}
//---------------------------------------------------------------------------------------------------------

void LcdDisplay(){
  #if defined(LCD)
    if(millis()-LcdRefresh[0]>LcdRefresh[1] || LcdNeedRefresh == true){
      lcd.setCursor(0,0);
      // lcd.print("ANT ");
      #if defined(WATCHDOG)
        if((millis() - WatchdogTimeout[0]) > WatchdogTimeout[1]) {
          lcd.print("* Timeout *");
        }else{
      #endif
          lcd.print(String(ANTname[BAND]).substring(0, 11));   // crop up to 7 char
          Space(11, String(ANTname[BAND]).length(), ' ');
      #if defined(WATCHDOG)
        }
      #endif

      lcd.setCursor(11,0);
      lcd.print(" ");
      lcd.print(BCDmatrixOUT[3][BAND]);
      lcd.print(BCDmatrixOUT[2][BAND]);
      lcd.print(BCDmatrixOUT[1][BAND]);
      lcd.print(BCDmatrixOUT[0][BAND]);

      lcd.setCursor(0,1);
      lcd.print("B");
      Space(2, String(BAND).length(), '-');
      lcd.print(BAND);
      lcd.print(" ");

      #if !defined(YAESU_BCD) && !defined(ICOM_ACC)
        Space(7, String(freq/1000).length(), ' ');
        PrintFreq();
      #endif
        if(freq<100000000){
          lcd.setCursor(4, 1);
        }else{
          lcd.setCursor(3, 1);
        }
        if(PTT==true){
          // lcd.write(byte(0));        // Lock icon
          lcd.print((char)0);
        }else{
          lcd.print(" ");        // Lock icon
        }
        #if !defined(YAESU_BCD) && !defined(ICOM_ACC)
          lcd.setCursor(13,1);
          lcd.print("kHz");
        #endif
        #if defined(ICOM_ACC)
          lcd.setCursor(10,1);
          lcd.print(VOLTAGE);
          lcd.print(" V");
        #endif

      LcdRefresh[0]=millis();
      LcdNeedRefresh = false;
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

void PrintFreq(){
  int longer=String(freq/1000).length();
  if(longer<4){
    lcd.print(" ");
    lcd.print(freq);
  }else{
    lcd.print(String(freq/1000).substring(0, longer-3));
    lcd.print(".");
    lcd.print(String(freq/1000).substring(longer-3, longer));
  }
}
//---------------------------------------------------------------------------------------------------------

void EthernetServer(){
  #if defined(EthModule)
    EthernetClient client = server.available();
    if (client) {
      boolean currentLineIsBlank = true;
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          HTTP_req += c;
          if (c == '\n' && currentLineIsBlank) {
            client.println(F("HTTP/1.1 200 OK"));
            client.println(F("Content-Type: text/html"));
            client.println(F("Connection: close"));
            client.println();
            client.println(F("<!DOCTYPE html>"));
            client.println(F("<html>"));
            client.println(F("<head>"));
            client.print(F("<title>"));
            client.println(F("Band Decoder</title>"));
            client.print(F("<meta http-equiv=\"refresh\" content=\"10;url=http://"));
            client.print(Ethernet.localIP());
            client.println(F("\">"));
            client.println(F("<link href='http://fonts.googleapis.com/css?family=Roboto+Condensed:300italic,400italic,700italic,400,700,300&subset=latin-ext' rel='stylesheet' type='text/css'>"));
            client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"));
            client.println(F("<meta name=\"mobile-web-app-capable\" content=\"yes\">"));
            client.println(F("<style type=\"text/css\">"));
            client.println(F("body {font-family: 'Roboto Condensed',sans-serif,Arial,Tahoma,Verdana;background: #ccc;}"));
            client.println(F("a:link  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("a:visited  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("a:hover  {color: #888;font-weight: bold;text-decoration: none;}"));
            client.println(F("input {border: 2px solid #ccc;background: #fff;margin: 10px 5px 0 0;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #333;}"));
            client.println(F("input:hover {border: 2px solid #080;}"));
            client.println(F("input.g {background: #080;color: #fff;}"));
            client.println(F("input.gr {background: #800;color: #fff;}"));
            client.println(F(".box {border: 2px solid #080;background: #ccc;  line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
            client.println(F(".boxr {border: 2px solid #800;background: #ccc; line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
            // client.println(F(".ptt {border: 2px solid #800;background: #ccc;margin: 10px 15px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #800;}"));
            client.println(F("</style></head><body><p>Input TRX <span class=\"boxr\">"));
            #if defined(INPUT_SERIAL)
              client.print(F("Input serial"));
            #endif
            #if defined(ICOM_CIV)
              client.print(F("ICOM CI-V</span><br>CI-V address <span class=\"box\">"));
              client.print(CIV_ADRESS);
              client.print(F("h"));
            #endif
            #if defined(KENWOOD_PC)
              client.print(F("KENWOOD"));
            #endif
            #if defined(YAESU_CAT)
              client.print(F("YAESU"));
            #endif
            #if defined(YAESU_CAT_OLD)
              client.print(F("YAESU [Old]"));
            #endif
            #if defined(YAESU_BCD)
              client.print(F("BCD"));
            #endif
            #if defined(ICOM_ACC)
              client.print(F("ICOM ACC voltage"));
            #endif
            client.print(F("</span><br>Baudrate <span class=\"box\">"));
            client.print(SERBAUD);
            client.print(F("</span><br>Watchdog second <span class=\"box\">"));
            #if defined(WATCHDOG)
              client.print(WATCHDOG);
              client.print(F("</span><br>Request <span class=\"box\">"));
            #endif
            client.print(REQUEST);
            client.print(F("ms</span><br>Band <span class=\"box\">"));
            client.print(BAND);
            client.print(F("</span><br>Frequency <span class=\"boxr\">"));
            client.print(freq);
            client.print(F("Hz</span><br>Power voltage: <span class=\"box\">"));
            client.print(volt(analogRead(VoltagePin)));
            client.println(F(" V</span></p>"));
            client.println(F("<p><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=220,height=350,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a></p>"));
            client.println(F("</body></html>"));

            // Serial.print(HTTP_req);
            HTTP_req = "";
            break;
          }
          if (c == '\n') {
            currentLineIsBlank = true;
          }
          else if (c != '\r') {
            currentLineIsBlank = false;
          }
        }
      }
      delay(1);
      client.stop();
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

#if defined(EthModule)
  void GetBoardId(){
    BOARD_ID = 0;
    if(digitalRead(Id1Pin)==0){
      BOARD_ID = BOARD_ID | (1<<0);    // Set the n-th bit
    }
    if(digitalRead(Id2Pin)==0){
      BOARD_ID = BOARD_ID | (1<<1);    // Set the n-th bit
    }
    if(digitalRead(Id3Pin)==0){
      BOARD_ID = BOARD_ID | (1<<2);    // Set the n-th bit
    }
    // if(digitalRead(Id4Pin)==0){
    //   BOARD_ID = BOARD_ID | (1<<3);    // Set the n-th bit
    // }
  }
#endif
//---------------------------------------------------------------------------------------------------------

float volt(int raw, float divider) {
  // float voltage = (raw * 5.0) / 1024.0 * ResistorCoeficient;
  float voltage = float(raw) * ArefVoltage * divider / 1023.0;
  #if defined(serialECHO)
    Serial.print("Voltage ");
    Serial.println(voltage);
  #endif
  return voltage;
}
//-------------------------------------------------------------------------------------------------------

void DCinMeasure(){
  if (millis() - VoltageRefresh[0] > VoltageRefresh[1]){
    DCinVoltage = volt(analogRead(VoltagePin), ResistorCoeficient);
    if (DCinVoltage<7){
      lcd.setCursor(3, 0);
      lcd.print(" Power LOW!");
    }else if (DCinVoltage>15){
      lcd.setCursor(2, 0);
      lcd.print("Power HIGH!");
    }
    VoltageRefresh[0] = millis();                      // set time mark
  }
}
//---------------------------------------------------------------------------------------------------------

void BandDecoderInput(){
  #if !defined(YAESU_BCD)
    InterruptON(0,0); // ptt, enc
  #endif

  //----------------------------------- Input Serial
  #if defined(INPUT_SERIAL)
    while (Serial.available() > 0) {
        BAND = Serial.parseInt();
        freq = Serial.parseInt();
        if (Serial.read() == '\n') {
            bandSET();
            #if defined(SERIAL_echo)
                serialEcho();
            #endif
        }
    }
  #endif

  //----------------------------------- Icom ACC
  #if defined(ICOM_ACC)
    VALUE = analogRead(ADPin);
    #if defined(ENABLE_DIVIDER)
      Divider = 6;
    #endif
    if (counter == 5) {
        VOLTAGE = float(VALUE) * ArefVoltage * Divider / 1023.0;

        //=====[ Icom ACC voltage range ]===========================================================

        if (VOLTAGE > 0.73 && VOLTAGE < 1.00 ) {BAND=10;}  //   6m   * * * * * * * * * * * * * * * *
        if (VOLTAGE > 1.00 && VOLTAGE < 1.09 ) {BAND=9;}   //  10m   *           Need              *
        if (VOLTAGE > 1.09 && VOLTAGE < 1.32 ) {BAND=8;}   //  12m   *    calibrated to your       *
        if (VOLTAGE > 1.32 && VOLTAGE < 1.55 ) {BAND=7;}   //  15m   *         own ICOM            *
        if (VOLTAGE > 1.55 && VOLTAGE < 1.77 ) {BAND=6;}   //  17m   *     ----------------        *
        if (VOLTAGE > 1.77 && VOLTAGE < 2.24 ) {BAND=5;}   //  20m   *    (These values have       *
        if (VOLTAGE > 0.10 && VOLTAGE < 0.50 ) {BAND=4;}   //  30m   *   been measured by any)     *
        if (VOLTAGE > 2.24 && VOLTAGE < 2.73 ) {BAND=3;}   //  40m   *          ic-746             *
        if (VOLTAGE > 2.73 && VOLTAGE < 2.99 ) {BAND=2;}   //  80m   *                             *
        if (VOLTAGE > 2.99 && VOLTAGE < 4.00 ) {BAND=1;}   // 160m   * * * * * * * * * * * * * * * *
        if (VOLTAGE > 0.00 && VOLTAGE < 0.10 ) {BAND=0;}   // parking

        //==========================================================================================

        bandSET();                                // set outputs
        delay (20);
    }else{
        if (abs(prevVALUE-VALUE)>10) {            // average
            //means change or spurious number
            prevVALUE=VALUE;
        }else {
            counter++;
            prevVALUE=VALUE;
        }
    }
    #if defined(SERIAL_echo)
        serialEcho();
        Serial.print(VOLTAGE);
        Serial.println(" V");
        Serial.flush();
    #endif

    delay(500);                                   // refresh time
  #endif

  //----------------------------------- Icom CIV
  #if defined(ICOM_CIV)
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        icomSM(incomingByte);
        rdIS="";
        if(rdI[10]==0xFD){                    // state machine end
          for (int i=9; i>=5; i-- ){
              if (rdI[i] < 10) {            // leading zero
                  rdIS = rdIS + 0;
              }
              rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
          }
          freq = rdIS.toInt();
          FreqToBandRules();
          bandSET();

          #if defined(SERIAL_echo)
              serialEcho();
          #endif
          RequestTimeout[0]=millis();
        }
    }
  #endif

  //----------------------------------- Yaesu BCD
  #if defined(YAESU_BCD)
    if (millis() - BcdInRefresh[0] > BcdInRefresh[1]){
      BAND = 0;
      if(digitalRead(BcdIn1Pin)==0){
        BAND = BAND | (1<<3);    // Set the n-th bit
      }
      if(digitalRead(BcdIn2Pin)==0){
        BAND = BAND | (1<<2);
      }
      if(digitalRead(BcdIn3Pin)==0){
        BAND = BAND | (1<<1);
      }
      if(digitalRead(BcdIn4Pin)==0){
        BAND = BAND | (1<<0);
      }
      bandSET();
      #if defined(SERIAL_echo)
          serialEcho();
      #endif
      BcdInRefresh[0]=millis();
    }
  #endif

  //----------------------------------- Kenwood
  #if defined(KENWOOD_PC)
    // Data exapmple
    // IF00007151074      000000000030000080;
    // IF00007032327      000000000030000080;
    while (Serial.available()) {
        rdKS="";
        Serial.readBytesUntil(lf, rdK, 38);       // fill array from serial
            if (rdK[0] == 73 && rdK[1] == 70){     // filter
                for (int i=2; i<=12; i++){          // 3-13 position to freq
                    rdKS = rdKS + String(rdK[i]);   // append variable to string
                }
                freq = rdKS.toInt();
                FreqToBandRules();
                bandSET();                                              // set outputs relay

                #if defined(SERIAL_echo)
                    serialEcho();
                #endif
            }
            memset(rdK, 0, sizeof(rdK));   // Clear contents of Buffer
    }
  #endif

  //----------------------------------- Yaesu CAT
  #if defined(YAESU_CAT)
      #include "yaesu_cat.h"
  #endif
  #if defined(YAESU_CAT_OLD)
      #include "yaesu_cat_old.h"
  #endif

  #if !defined(YAESU_BCD)
    InterruptON(1,1); // ptt, enc
  #endif
}
//---------------------------------------------------------------------------------------------------------

void BandDecoderOutput(){

  //=====[ Output Remote relay ]=======================
  #if defined(REMOTE_RELAY)
      timeout3 = millis()-previous3;                  // check timeout
      if (timeout3>(watchdog3)){
          remoteRelay();
          previous3 = millis();                       // set time mark
      }
  #endif
  //=====[ Output Icom CIV ]=======================
  #if defined(ICOM_CIV_OUT)
      if(freq!= freqPrev1){                    // if change
          txCIV(0, freq, CIV_ADR_OUT);         // 0 - set freq
          freqPrev1 = freq;
      }
  #endif
  //=====[ Output Kenwood PC ]=====================
  #if !defined(REQUEST) && defined(KENWOOD_PC_OUT)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 11) {       // leding zeros
              freqPCtx = 0 + freqPCtx;
          }
         Serial.print("FA" + freqPCtx + ";");    // sets both VFO
         Serial.print("FB" + freqPCtx + ";");
//          Serial.print("FA" + freqPCtx + ";");    // first packet not read every time
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif
  //=====[ Output Yaesu CAT ]=====================
  #if !defined(REQUEST) && defined(YAESU_CAT_OUT)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 8) {        // leding zeros
              freqPCtx = 0 + freqPCtx;
          }
         Serial.print("FA" + freqPCtx + ";");    // sets both VFO
         Serial.print("FB" + freqPCtx + ";");
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif
  //=====[ Output Yaesu CAT OLD ]=================
  #if !defined(REQUEST) && defined(YAESU_CAT_OUT_OLD)
      if(freq != freqPrev2){                     // if change
          String freqPCtx = String(freq);        // to string
          while (freqPCtx.length() < 8) {        // leding zeros
              freqPCtx = 0 + freqPCtx;
         }
         Serial.write(1);                        // set freq
         Serial.flush();
         freqPrev2 = freq;
      }
  #endif

}
//---------------------------------------------------------------------------------------------------------

void bandSET() {                                               // set outputs by BAND variable
  if(BAND==0 && previousBAND != 0){    // deactivate PTT
    digitalWrite(PttOffPin, HIGH);
    PTT = true;
    LcdNeedRefresh = true;
  }
  if(PTT==false){
    ShiftByte[0] = B00000000;
    ShiftByte[1] = B00000000;

    if(BAND > 0 && BAND < 9){
      ShiftByte[0] = ShiftByte[0] | (1<<BAND-1);    // Set the n-th bit
    }
    if(BAND > 7 && BAND < 17){
      ShiftByte[1] = ShiftByte[1] | (1<<BAND-9);    // Set the n-th bit
    }
    digitalWrite(ShiftOutLatchPin, LOW);    // ready for receive data
    if(NumberOfBoards > 1){ shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[1]); }
                            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftByte[0]);
    digitalWrite(ShiftOutLatchPin, HIGH);    // switch to output pin

      #if !defined(YAESU_BCD)
          bcdOut();
      #endif
      #if defined(EthModule)
        RemoteSwIP = DetectedRemoteSw[BAND];
        RemoteSwPort = DetectedRemoteSw[BAND][4];

        Serial.print(RemoteSwIP);
        Serial.print(":");
        Serial.println(RemoteSwPort);

        // UDP send to Switch
        TxUdpBuffer[0] = B01110011;         // s
        TxUdpBuffer[1] = B00111010;         // :
        TxUdpBuffer[2] = 0; //rxShiftInButton[0];  // set buffer
        TxUdpBuffer[3] = 1;  //rxShiftInButton[1];
        TxUdpBuffer[4] = 2;  //rxShiftInButton[2];
        TxUdpBuffer[5] = B00111011;         // ;
        UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          RemoteSwLatency[0] = millis(); // set START time mark UDP command latency
        UdpCommand.endPacket();
        RemoteSwLatencyAnsw = 0;   // send command, wait to answer

      #endif
      LcdNeedRefresh = true;
      previousBAND = BAND;
  }
  #if defined(WATCHDOG)
    WatchdogTimeout[0] = millis();                   // set time mark
  #endif
}
//---------------------------------------------------------------------------------------------------------

void remoteRelay() {
    Serial.print(1);
    Serial.print(',');
    Serial.print(BAND, DEC);
    Serial.print('\n');
    Serial.flush();
}
//---------------------------------------------------------------------------------------------------------

void serialEcho() {
    Serial.print("<");
    Serial.print(BAND);
    Serial.print(",");
    Serial.print(freq);
    Serial.println(">");
    Serial.flush();
}
//---------------------------------------------------------------------------------------------------------

#if !defined(YAESU_BCD)
    void bcdOut(){
        if (BCDmatrixOUT[0][BAND] == 1){ digitalWrite(BcdIn1Pin, HIGH); }else{ digitalWrite(BcdIn1Pin, LOW);}
        if (BCDmatrixOUT[1][BAND] == 1){ digitalWrite(BcdIn2Pin, HIGH); }else{ digitalWrite(BcdIn2Pin, LOW);}
        if (BCDmatrixOUT[2][BAND] == 1){ digitalWrite(BcdIn3Pin, HIGH); }else{ digitalWrite(BcdIn3Pin, LOW);}
        if (BCDmatrixOUT[3][BAND] == 1){ digitalWrite(BcdIn4Pin, HIGH); }else{ digitalWrite(BcdIn4Pin, LOW);}
    }
#endif
//---------------------------------------------------------------------------------------------------------

void watchDog() {
  #if defined(WATCHDOG)
    if((millis() - WatchdogTimeout[0]) > WatchdogTimeout[1]) {
      BAND=0;
      freq=0;
      bandSET();                                 // set outputs
    }
  #endif
}
//---------------------------------------------------------------------------------------------------------

#if defined(ICOM_CIV) || defined(ICOM_CIV_OUT)

    int icomSM(byte b){      // state machine
        // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
        switch (state) {
            case 1: if( b == 0xFE ){ state = 2; rdI[0]=b; }; break;
            case 2: if( b == 0xFE ){ state = 3; rdI[1]=b; }else{ state = 1;}; break;
            // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
            case 3: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 4; rdI[2]=b;                             // choose command $03
              }else if( b == CIV_ADRESS ){ state = 6; rdI[2]=b;}else{ state = 1;}; break;                       // or $05

            case 4: if( b == CIV_ADRESS ){ state = 5; rdI[3]=b; }else{ state = 1;}; break;                      // select command $03
            case 5: if( b == 0x00 || b == 0x03 ){state = 8; rdI[4]=b; }else{ state = 1;}; break;

            case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 7; rdI[3]=b; }else{ state = 1;}; break;  // select command $05
            case 7: if( b == 0x00 || b == 0x05 ){ state = 8; rdI[4]=b; }else{ state = 1;}; break;

            case 8: if( b <= 0x99 ){state = 9; rdI[5]=b; }else{state = 1;}; break;
            case 9: if( b <= 0x99 ){state = 10; rdI[6]=b; }else{state = 1;}; break;
           case 10: if( b <= 0x99 ){state = 11; rdI[7]=b; }else{state = 1;}; break;
           case 11: if( b <= 0x99 ){state = 12; rdI[8]=b; }else{state = 1;}; break;
           case 12: if( b <= 0x99 ){state = 13; rdI[9]=b; }else{state = 1;}; break;
           case 13: if( b == 0xFD ){state = 1; rdI[10]=b; }else{state = 1; rdI[10] = 0;}; break;
        }
    }
    //---------------------------------------------------------------------------------------------------------

    int txCIV(int commandCIV, long dataCIVtx, int toAddress) {
        //Serial.flush();
        Serial.write(254);                                    // FE
        Serial.write(254);                                    // FE
        Serial.write(toAddress);                              // to adress
        Serial.write(fromAdress);                             // from OE
        Serial.write(commandCIV);                             // data
        if (dataCIVtx != 0){
            String freqCIVtx = String(dataCIVtx);             // to string
            String freqCIVtxPart;
            while (freqCIVtx.length() < 10) {                 // leding zeros
                freqCIVtx = 0 + freqCIVtx;
            }
            for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
                freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                    Serial.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
            }
        }
        Serial.write(253);                                    // FD
        // Serial.flush();
        while(Serial.available()){        // clear buffer
          Serial.read();
        }
    }
    //---------------------------------------------------------------------------------------------------------

    unsigned int hexToDec(String hexString) {
        unsigned int decValue = 0;
        int nextInt;
        for (int i = 0; i < hexString.length(); i++) {
            nextInt = int(hexString.charAt(i));
            if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
            if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
            if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
            nextInt = constrain(nextInt, 0, 15);
            decValue = (decValue * 16) + nextInt;
        }
        return decValue;
    }
    //---------------------------------------------------------------------------------------------------------
#endif

void FreqToBandRules(){
         if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
    else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
    else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
    else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
    else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
    else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
    else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
    else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
    else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
    else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
    else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  //   2m
    else {BAND=0;}                                                // out of range
}
