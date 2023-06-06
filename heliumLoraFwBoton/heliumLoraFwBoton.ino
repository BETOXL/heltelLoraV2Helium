/*
 * HelTec Automation(TM) LoRaWAN 1.0.2 OTAA example use OTAA, CLASS A
 *
 * Function summary:
 *
 * - You can use port  to control the LED light.
 *
 * - If the issued value is 1(ASCII), the lamp will be turned off Policia.
 * - The release value is 2(ASCII) and the light will be turned off Ambulancia.
 * - The release value is 3(ASCII) and the light will be turned off Bomberos.
 *
 * - use internal RTC(150KHz);
 *
 * - Include stop mode and deep sleep mode;
 *
 * - 15S data send cycle;
 *
 * - Informations output via serial(115200);
 *
 * - Only ESP32 + LoRa series boards can use this library, need a license
 *   to make the code run(check you license here: http://www.heltec.cn/search/);
 *
 * You can change some definition in "Commissioning.h" and "LoRaMac-definitions.h"
 *
 * HelTec AutoMation, Chengdu, China.
 * 成都惠利特自动化科技有限公司
 * https://heltec.org
 * support@heltec.cn
 *
 *this project also release in GitHub:
 *https://github.com/HelTecAutomation/ESP32_LoRaWAN
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"

/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = { 0x5F056ED9, 0x527C8AA4, 0x73FE696A, 0x3C83FCFB };

/* OTAA para AvAlberdi1235*/

uint8_t DevEui[] = { 0x60, 0x81, 0xF9, 0xD2, 0x11, 0x0D, 0x94, 0xF6 };
uint8_t AppEui[] = { 0x60, 0x81, 0xF9, 0x67, 0x3E, 0x24, 0xE5, 0xA4 };
uint8_t AppKey[] = { 0x3A, 0xE5, 0x2F, 0xF6, 0x46, 0x2A, 0xBA, 0x3F, 0x43, 0xD7, 0x87, 0x65, 0x36, 0x69, 0x06, 0xCC };

/* ABP para NO SE USA*/
uint8_t NwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00 };
uint8_t AppSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00 };
uint32_t DevAddr =  ( uint32_t )0x00000000;

/*LoraWan channelsmask, default channels 0-7*/ 

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].
uint32_t appTxDutyCycle = 15000;*/
uint32_t appTxDutyCycle = 3600*24*1000;
/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
* None : print basic info.
* Freq : print Tx and Rx freq, DR info.
* Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
* Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt and MCU deepsleep info.
*/
uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;


#define LEDPin_policia      17 //LED policia
#define LEDPin_ambulancia   12  //LED ambulancia
#define LEDPin_bomberos     13  //LED bomberos

#define INT_PIN_policia     36 //button policia
#define INT_PIN_ambulancia  37 //button ambulancia
#define INT_PIN_bomberos    38 //button bomberos
#define Buzzer              25
bool isPolicia = false;
bool isAmbulancia = false;
bool isBombero = false;

bool estPolicia = false;
bool estAmbulancia = false;
bool estBombero = false;

void app(uint8_t data)
 {
    // lora_printf("data:%d\r\n",data);
   switch(data)
     {
    case 49:
    {
      isPolicia = false;
      break;
    }
    case 50:
    {
      isAmbulancia = false;
      break;
    }
    case 51:
    {
      isBombero = false;
      break;
    }
    default:
    {
      break;
    }
     }
 }


void  downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  lora_printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  lora_printf("+REV DATA:");
    app(mcpsIndication->Buffer[0]);

  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    lora_printf("%02X",mcpsIndication->Buffer[i]);
  }
  lora_printf("\r\n");
}

void IRAM_ATTR  keyDownPolicia()
{

  if(digitalRead(INT_PIN_policia)== LOW) 
  {
    isPolicia = true;
  }
}

void IRAM_ATTR  keyDownAmbulancia()
{

  if(digitalRead(INT_PIN_ambulancia)== LOW) 
  {
    isAmbulancia = true;
  }
}

void IRAM_ATTR  keyDownBomberos()
{

  if(digitalRead(INT_PIN_bomberos)== LOW) 
  {
    isBombero = true;
  }
}
static void prepareTxFrame( uint8_t port )
{
    appDataSize = 3;//AppDataSize max value is 64
    if(isPolicia){
      appData[0] = 0x49;
    }else{
      appData[0] = 0x00;
    }

    if(isAmbulancia){
      appData[1] = 0x50;
    }else{
      appData[1] = 0x00;
    }

    if(isBombero){
      appData[2] = 0x51;
    }else{
      appData[2] = 0x00;
    }
    
    
}

// Add your initialization code here
void setup()
{
  if(mcuStarted==0)
  {
    LoRaWAN.displayMcuInit();
  }
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK,MISO,MOSI,SS);
  Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
  delay(100);
  pinMode(LEDPin_policia,OUTPUT);
  pinMode(INT_PIN_policia,INPUT_PULLUP);
  attachInterrupt(INT_PIN_policia,keyDownPolicia,FALLING);
  
  pinMode(LEDPin_ambulancia,OUTPUT);
  pinMode(INT_PIN_ambulancia,INPUT_PULLUP);
  attachInterrupt(INT_PIN_ambulancia,keyDownAmbulancia,FALLING);

  pinMode(LEDPin_bomberos,OUTPUT);
  pinMode(INT_PIN_bomberos,INPUT_PULLUP);
  attachInterrupt(INT_PIN_bomberos,keyDownBomberos,FALLING);

  pinMode(Buzzer,OUTPUT);
  
  deviceState = DEVICE_STATE_INIT;
}

// The loop function is called in an endless loop
void loop()
{
  
  if(isPolicia != estPolicia){
    estPolicia=isPolicia;
    if(isPolicia){
      tone(Buzzer,1000, 500);
      digitalWrite(LEDPin_policia, HIGH);
      if(IsLoRaMacNetworkJoined){
        deviceState = DEVICE_STATE_SEND;
      }
    }else{
      digitalWrite(LEDPin_policia, LOW);
    }
  }
  
  if(isAmbulancia != estAmbulancia){
    estAmbulancia = isAmbulancia;
    if(isAmbulancia){
        tone(Buzzer,1000, 500);
        digitalWrite(LEDPin_ambulancia, HIGH);
        if(IsLoRaMacNetworkJoined){
          deviceState = DEVICE_STATE_SEND;
        }
    }else{
      digitalWrite(LEDPin_ambulancia, LOW);
    }
  }
  
  if(isBombero != estBombero){
    estBombero=isBombero;
    if(isBombero){
      tone(Buzzer,1000, 500);
      digitalWrite(LEDPin_bomberos, HIGH);
      if(IsLoRaMacNetworkJoined){
        deviceState = DEVICE_STATE_SEND;
      }
    }else{
      digitalWrite(LEDPin_bomberos, LOW);
    }
  }
  switch( deviceState )
  {
    case DEVICE_STATE_INIT:
    {
#if(LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass,loraWanRegion);
      break;
    }
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.displayJoining();
      LoRaWAN.join();
      break;
    }
    case DEVICE_STATE_SEND:
    {
      LoRaWAN.displaySending();
      prepareTxFrame( appPort );
      LoRaWAN.send(loraWanClass);
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.displayAck();
      LoRaWAN.sleep(loraWanClass,debugLevel);
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }

  
}
