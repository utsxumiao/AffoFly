
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include <RF24.h>
#include "NRF24_RX.h"

#if defined(NRF24_RX)

int16_t nrf24_rcData[RC_CHANS];

// Single radio pipe address for the 2 nodes to communicate.
static const uint64_t pipe = 0xE8E8F0F0E1LL;
const uint8_t RADIO_CHANNEL = 101;

RF24 radio(7, 8); // CE, CSN

RF24Data nrf24Data;
//RF24AckPayload nrf24AckPayload;
//extern RF24AckPayload nrf24AckPayload;

void resetRF24Data() 
{
  nrf24Data.Throttle = 1000;
  nrf24Data.Yaw = 1500;
  nrf24Data.Pitch = 1500;
  nrf24Data.Roll = 1500;
  nrf24Data.Aux1 = 1000;
  nrf24Data.Aux2 = 1000;
  nrf24Data.Aux3 = 1000;
  nrf24Data.Aux4 = 1000;
  nrf24Data.Aux5 = 1000;
  nrf24Data.Aux6 = 1000;
}

//void resetRF24AckPayload() 
//{
//  nrf24AckPayload.lat = 0;
//  nrf24AckPayload.lon = 0;
//  nrf24AckPayload.heading = 0;
//  nrf24AckPayload.pitch = 0;
//  nrf24AckPayload.roll = 0;
//  nrf24AckPayload.alt = 0;
//  nrf24AckPayload.flags = 0;
//}

void NRF24_Init() {

  resetRF24Data();
  //resetRF24AckPayload();

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(RADIO_CHANNEL);
  radio.setAutoAck(false);                    // Ensure autoACK is enabled
  //radio.enableAckPayload();

  radio.openReadingPipe(1,pipe);
  radio.startListening();  
}

void NRF24_Read_RC() {
  
  static unsigned long lastRecvTime = 0;

//  nrf24AckPayload.lat = 35.62;
//  nrf24AckPayload.lon = 139.68;
//  nrf24AckPayload.heading = att.heading;
//  nrf24AckPayload.pitch = att.angle[PITCH];
//  nrf24AckPayload.roll = att.angle[ROLL];
//  nrf24AckPayload.alt = alt.EstAlt;
//  memcpy(&nrf24AckPayload.flags, &f, 1); // first byte of status flags
	
  unsigned long now = millis();
  while ( radio.available() ) {
    //radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
    radio.read(&nrf24Data, sizeof(RF24Data));
    lastRecvTime = now;
  }
  if ( now - lastRecvTime > 500 ) {
    // signal lost?
    resetRF24Data();
  }
  
  nrf24_rcData[THROTTLE] = nrf24Data.Throttle;
  nrf24_rcData[YAW] =      nrf24Data.Yaw;
  nrf24_rcData[PITCH] =    nrf24Data.Pitch;
  nrf24_rcData[ROLL] =     nrf24Data.Roll;
  nrf24_rcData[AUX1] =     nrf24Data.Aux1;
  nrf24_rcData[AUX2] =     nrf24Data.Aux2;
  nrf24_rcData[AUX3] =     nrf24Data.Aux3;
  nrf24_rcData[AUX4] =     nrf24Data.Aux4;
//  nrf24_rcData[8] =        nrf24Data.Aux5;
//  nrf24_rcData[9] =        nrf24Data.Aux6;
}

#endif

