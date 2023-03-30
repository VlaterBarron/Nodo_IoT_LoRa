#include <Arduino.h>
#include "SHT1x.h"
#include "src\lmic.h"
#include "src\hal\hal.h"
#include "SPI.h"

//////////////////////////////////////////////////////////// Definición de pines //////////////////////////////////////////////////////////////////////
#define RFM95_RST 13 //P1.5
#define RFM95_CS  18 //P3.0
#define RFM95_DIO0 11//P1.3
#define RFM95_DIO1 12//P1.4
#define RFM95_DIO2 5 //P4.3
//#define CLOCK_PIN 9  //P3.5       
#define P_Bat A7     //P2.4 //Pin 6     
#define SCK 7        //P2.2    
#define MISO 14      //P1.7
#define RF_En 19     //P1.2
//Sensor Temperatura
//#define TempDataPin2 P4_3
//#define TempClockPin2 P2_2
#define TempDataPin1 P3_6
#define TempClockPin1 P3_5

/// bmp280
//      SDA   10     //P3.6
//      SCL    9     //P3.5
//Adafruit_BMP280 bmp;  //bmp280

//SHTx
//      SDA   10    //P3.6
//      SCL   9     //P3.5
SHT1x temp1(TempDataPin1, TempClockPin1);

//////////////////////////////////////////////////////////// Variables del programa //////////////////////////////////////////////////////////////////////
float bat_Level = 0; //?

const unsigned sleepsecs = 60; //Time in seconds the node will remain slept

const unsigned SLEEP_TIME = (sleepsecs/3.85); //Sleep_Time = (t-27.8)/3.38 s

static osjob_t sendjob;

//////////////////////////////////////////////////////////// Direcciones de los nodos //////////////////////////////////////////////////////////////////////
//ID: 00028
//LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = {0x79, 0x87, 0xE6, 0x61, 0x7A, 0xD9, 0xF5, 0x36, 0x5F, 0xDF, 0x63, 0x3C, 0xE8, 0x80, 0x43, 0xEE};
//LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0x43, 0xA1, 0x0E, 0xCB, 0x9A, 0x9C, 0xB7, 0xB7, 0xC5, 0x5B, 0x98, 0xE0, 0x84, 0x98, 0x25, 0x73};
//LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011B79;

//ID: 0001
//static const PROGMEM u1_t NWKSKEY[16] = { 0xD2, 0xFD, 0xD7, 0x9B, 0xA9, 0xAF, 0x6D, 0xA0, 0x74, 0x9D, 0x9A, 0x0F, 0x4D, 0xE0, 0x62, 0xA1 };
//static const u1_t PROGMEM APPSKEY[16] = { 0x1F, 0x85, 0xFD, 0x51, 0x75, 0xC9, 0xF7, 0x37, 0x3E, 0xE6, 0xB5, 0xFB, 0x30, 0x66, 0x8C, 0xEC };
//static const u4_t DEVADDR = 0x260119A2; // <-- Change this address for every node! mobilefish

void os_getArtEui (u1_t* buf){ }
void os_getDevEui (u1_t* buf){ }
void os_getDevKey (u1_t* buf){ }

//////////////////////////////////////////////////////////// LoRa //////////////////////////////////////////////////////////////////////

void LoRa(boolean S){
  if(S){
    digitalWrite(RF_En, HIGH);
    os_init();
    LMIC_reset();

#ifdef PROGMEM
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868) //Canales para el gateway
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
#elif defined(CFG_us915)
    LMIC_selectSubBand(1);
#endif

    LMIC_setAdrMode(1);
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF9;
    LMIC.txpow = 27;
    LMIC_setDrTxpow(DR_SF12, 14); //0db
  }else{
//      Serial.println(F("Sleep!"));
      hal_sleep();//      LMIC_shutdown();
      digitalWrite(RF_En, LOW);
  }
}

//////////////////////////////////////////////////////////// Clock Settings //////////////////////////////////////////////////////////////////////

void Clock_Settings(){
  P2DIR |= BIT0;
  P2SEL0 |= BIT0;
  P2SEL1 |= BIT0;
  P3DIR |= BIT4;
  P3SEL0 |= BIT4;
  P3SEL1 |= BIT4;
  PJSEL0 |= BIT4 | BIT5 | BIT6 | BIT7;

  FRCTL0 = FRCTLPW | NWAITS_1;

  PM5CTL0 &= ~LOCKLPM5;

  CSCTL0_H = CSKEY >> 8;
  CSCTL1 = DCORSEL | DCOFSEL_4;
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
  CSCTL0_H = 0;

//  Serial.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
}

void default_Clock_Settings(){
  CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
  CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
  CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = ACLK = VLO
  CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
  CSCTL0_H = 0;                             // Lock CS registers
}

void do_send(osjob_t* j){
  if(LMIC.opmode & OP_TXRXPEND){
//    Serial.println(F("OP_TXRXPEND, not sending"));
  }else{
  Clock_Settings();


  bat_Level = analogRead(P_Bat) * 0.048828; //12 bits //?
//

//  bmp.setSampling(Adafruit_BMP280::MODE_FORCED, //bmp280
//                  Adafruit_BMP280::SAMPLING_X1, // temperature
//                  Adafruit_BMP280::SAMPLING_X1, // pressure
//                  Adafruit_BMP280::FILTER_OFF   );

//  bmp.takeForcedMeasurement(); //Forced mode: sensor takes 1 measurement and goes to sleep

//  float celsius = bmp.readTemperature();
//  float humi = 0;
//  float presr = bmp.readPressure()/4; // Pa/4 to send in 8 bits

int celsius = ((int)(temp1.readTemperatureC()*100))/100;
float humi = temp1.readHumidity();
float presr = 0;

  sleep(100);

  u2_t slp = (u2_t)(10 * 100);
  u2_t temperature = (u2_t)(celsius * 100);
  u2_t humidity = (u2_t)(humi * 100);
  u2_t prsatm = (u2_t)(presr * 100);
  u2_t bat_level_I = (u2_t)(bat_Level * 100);

  LMIC.frame[0] = (temperature >> 8) & 0xFF;
  LMIC.frame[1] = temperature & 0xFF;
  LMIC.frame[2] = (temperature >> 8) & 0xFF;
  LMIC.frame[3] = temperature & 0xFF;
  LMIC.frame[4] = (humidity >> 8) & 0xFF;
  LMIC.frame[5] = humidity & 0xFF;
  LMIC.frame[6] = (prsatm >> 8) & 0xFF;
  LMIC.frame[7] = prsatm & 0xFF;
  LMIC.frame[8] = (bat_level_I >> 8) & 0xFF;
  LMIC.frame[9] = bat_level_I & 0xFF;

  sleep(100);
  default_Clock_Settings();
  LMIC_setTxData2(1, LMIC.frame, 10, 0); // (port 1, data, 4 bytes, unconfirmed)
  }
}

//void SensorSHT();
//void SensorSHT()
//{
// float celsius = ((int)(temp1.readTemperatureC()*100))/100;
//float humi = temp1.readHumidity();
//float presr = 0;
//float temperature1 = ((int)(temp1.readTemperatureC()*100))/100;
//Serial.print("Temperatura: ");
//Serial.print(temperature1, 2);
//Serial.println(" °C");
//}


const lmic_pinmap lmic_pins = {
  RFM95_CS,   //nss
  LMIC_UNUSED_PIN,    //rxtx
  RFM95_RST,  //rst
  {RFM95_DIO0, RFM95_DIO1, RFM95_DIO2},  //DIO0,DIO1,DIO2 ???
};

//////////////////////////////////////////////////////////// EVENTS //////////////////////////////////////////////////////////////////////
void onEvent(ev_t ev){
  switch(ev){
    case EV_SCAN_TIMEOUT:
      break;
    case EV_BEACON_FOUND:
      break;
    case EV_BEACON_MISSED:
      break;
    case EV_BEACON_TRACKED:
      break;
    case EV_JOINING:
      break;
    case EV_JOINED:
      LMIC_setLinkCheckMode(0);
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||   Serial.println(F("EV_RFU1"));
      ||   break;
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_JOIN_FAILED:
      //      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      //      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:

      LoRa(false);
//      sleepSeconds(SLEEP_TIME*3.85);
//      os_setCallback(&sendjob, do_send);
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(SLEEP_TIME), do_send);
      LoRa(true);

      break;
    case EV_LOST_TSYNC:
      break;
    case EV_RESET:
      break;
    case EV_RXCOMPLETE:
      break;
    case EV_LINK_DEAD:
      break;
    case EV_LINK_ALIVE:
      break;
    default:
      break;
  }
}

//////////////////////////////////////////////////////////// VOID SETUP //////////////////////////////////////////////////////////////////////
void setup(){
  sleepSeconds(SLEEP_TIME*3.85); //to avoid sending data at beggining for 5 mins

  ///////////// Inicialización de pines /////////////
  pinMode(MISO, INPUT_PULLUP);
  pinMode(SCK, OUTPUT);
  pinMode(RFM95_CS, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RF_En, OUTPUT);
  pinMode(TempDataPin1,INPUT);
  pinMode(TempClockPin1,INPUT);
  /////////// Parámetros de comunicación  /////////////////
  Serial.begin(115200);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  digitalWrite(RFM95_CS, HIGH);

  digitalWrite(RF_En, LOW);

 // bmp.begin(); //bmp280

  LoRa(true);
  do_send(&sendjob);
}

//////////////////////////////////////////////////////////// VOID LOOP //////////////////////////////////////////////////////////////////////
void loop(){
os_runloop_once();

}