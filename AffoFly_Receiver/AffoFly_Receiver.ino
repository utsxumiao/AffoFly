#include <EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

//==== System Setting =============================================//
#define VERSION_NUMBER     "0.012"
#define DEBUG_MODE  1
#define CLOCK_MULTIPLIER 1 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino
//=======================================================================//

//==== Pin Definition ===================================================//
#define PPM_PIN       2 // It has to be 2 unless you also update setPPM() and ISR
#define BLUE_LED_PIN  3
#define RED_LED_PIN   4
#define NRF_CE_PIN    7
#define NRF_CSN_PIN   8
//=======================================================================//

//==== Global Constant & Variable =======================================//
const uint16_t PPM_FRAME_LENGTH = 24000;
const uint16_t PPM_PULSE_LENGTH = 400;
const uint8_t CHANNEL_COUNT = 10;
const uint16_t RADIO_TX_ID = 1005;
const uint16_t RADIO_RX_ID = 1;
const uint8_t RADIO_CHANNEL = 101;
const uint64_t RADIO_PIPE_OUT = 0xE8E8F0F0E1LL;

const uint16_t RADIO_SIGNAL_TIMEOUT = 500;
unsigned long RadioSignalLastMillis = 0;
const uint8_t LED_FLASH_INTERVAL = 20;
unsigned long LedFlashLastMillis = 0;
const uint16_t RATE_REFRESH_INTERVAL = 1000;
unsigned long RateRefreshLastMillis = 0;

bool HasSignal = false;
bool BlueLedState = false;
bool RedLedState = false;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

struct MyData {
  uint16_t TxId;
  //uint16_t RxId;
  uint16_t Throttle;
  uint16_t Yaw;
  uint16_t Pitch;
  uint16_t Roll;
  uint16_t Aux1;
  uint16_t Aux2;
  uint16_t Aux3;
  uint16_t Aux4;
  uint16_t Aux5;
  uint16_t Aux6;
};
MyData data;

uint16_t PPM[CHANNEL_COUNT];

#ifdef DEBUG_MODE
uint16_t PackageCount = 0;
uint16_t PackageRate = 0;
uint16_t LoopCount = 0;
uint16_t LoopRate = 0;
#endif
//=======================================================================//

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
  pinMode(PPM_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);
  radio.openReadingPipe(1, RADIO_PIPE_OUT);
  radio.startListening();
#ifdef DEBUG_MODE
  printf_begin();
  radio.printDetails();
#endif

  setupPPM();
  resetData();
}

void loop() {
  unsigned long currentMillis = millis();
  receiveData(currentMillis);
  checkSignal(currentMillis);
  flashLed(currentMillis);
  LoopCount++;
  calculateRate(currentMillis);
}

void calculateRate(unsigned long currentMillis) {
  if (currentMillis - RateRefreshLastMillis >= RATE_REFRESH_INTERVAL) {
    RateRefreshLastMillis = currentMillis;
    PackageRate = PackageCount;
    LoopRate = LoopCount;
    PackageCount = 0;
    LoopCount = 0;
#ifdef DEBUG_MODE
    Serial.print(PackageRate);  Serial.print(" / ");  Serial.println(LoopRate);
#endif
  }
}

void receiveData(unsigned long currentMillis) {
  while(radio.available()){
    radio.read(&data, sizeof(MyData));
    if(data.TxId == RADIO_TX_ID) {
      HasSignal = true;
      RadioSignalLastMillis = currentMillis;
      PackageCount++;
#ifdef DEBUG_MODE
//      Serial.print("Data received at: ");  Serial.print(millis());  Serial.print("    "); 
#endif
      setPPMValuesFromData();
    }
#ifdef DEBUG_MODE
//    Serial.print("PPM output at: ");  Serial.println(millis());
//    Serial.print("TxId: "); Serial.print(data.TxId);  Serial.print("    ");
//    Serial.print("Throttle: "); Serial.print(data.Throttle);  Serial.print("    ");
//    Serial.print("Yaw: ");      Serial.print(data.Yaw);       Serial.print("    ");
//    Serial.print("Pitch: ");    Serial.print(data.Pitch);     Serial.print("    ");
//    Serial.print("Roll: ");     Serial.print(data.Roll);      Serial.print("    ");
//    Serial.print("Aux1: ");     Serial.print(data.Aux1);      Serial.print("    ");
//    Serial.print("Aux2: ");     Serial.print(data.Aux2);      Serial.print("    ");
//    Serial.print("Aux3: ");     Serial.print(data.Aux3);      Serial.print("    ");
//    Serial.print("Aux4: ");     Serial.print(data.Aux4);      Serial.print("    ");
//    Serial.print("Aux5: ");     Serial.print(data.Aux5);      Serial.print("    ");
//    Serial.print("Aux6: ");     Serial.print(data.Aux6);      Serial.print("\n");
#endif
  }
}

void checkSignal(unsigned long currentMillis) {
  if(currentMillis - RadioSignalLastMillis >= RADIO_SIGNAL_TIMEOUT) {
    HasSignal = false;
    resetData();
  }
}

// Given currently the AffoFly TX (with 8mhz) sends about 70 pps
// I consider 50 pps on FC/RX side is good enough
// PackageRate >= 50: Blue solid, Red off
// PackageRate >= 40: Blue flashing, Red off
// PackageRate >= 30: Blue flashing, Red flashing
// PackageRate < 30:  Blue off, Red solid
// No signal (500ms no package received): Blue off, Red off
void flashLed(unsigned long currentMillis) {
  if (currentMillis - LedFlashLastMillis >= LED_FLASH_INTERVAL) {
    LedFlashLastMillis = currentMillis;
    if (HasSignal) {
      if (PackageRate >= 50) {
        BlueLedState = true;
        RedLedState = false;
      } else if (PackageRate >= 40) {
        BlueLedState = !BlueLedState;
        RedLedState = false;
      } else if (PackageRate >= 30) {
        BlueLedState = !BlueLedState;
        RedLedState = !RedLedState;
      } else {
        BlueLedState = false;
        RedLedState = true;
      }
    } else {
      if (BlueLedState) BlueLedState = false;
      if (RedLedState) RedLedState = false;
    }
    digitalWrite(BLUE_LED_PIN, BlueLedState);
    digitalWrite(RED_LED_PIN, RedLedState);
  }
}

void resetData() {
  //Value 1000 - 2000, 1500 is middle/neutral
  data.Throttle = 1000;
  data.Yaw = 1500;
  data.Pitch = 1500;
  data.Roll = 1500;
  data.Aux1 = 1000;
  data.Aux2 = 1000;
  data.Aux3 = 1000;
  data.Aux4 = 1000;
  data.Aux5 = 1000;
  data.Aux6 = 1000;

  setPPMValuesFromData();
}

void setPPMValuesFromData(){
  PPM[0] = data.Throttle;
  PPM[1] = data.Yaw;
  PPM[2] = data.Pitch;
  PPM[3] = data.Roll;
  PPM[4] = data.Aux1;
  PPM[5] = data.Aux2;
  PPM[6] = data.Aux3;
  PPM[7] = data.Aux4;
  PPM[8] = data.Aux5;
  PPM[9] = data.Aux6;
}

void setupPPM() {
  //digitalWrite(PPM_PIN, 0);  //set the PPM signal pin to the default state (off)
  PORTD = PORTD & ~B00000100;
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  TCNT1 = 0;
#ifdef DEBUG_MODE
    //Serial.print("PPM-Throttle: ");     Serial.println(PPM[0]);
#endif
  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off.
    OCR1A = PPM_PULSE_LENGTH * CLOCK_MULTIPLIER;
    state = false;
  }else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on.
    state = true;

    if (cur_chan_numb >= CHANNEL_COUNT) {
      cur_chan_numb = 0;
      calc_rest += PPM_PULSE_LENGTH;
      OCR1A = (PPM_FRAME_LENGTH - calc_rest) * CLOCK_MULTIPLIER;
      calc_rest = 0;
    }
    else {
      OCR1A = (PPM[cur_chan_numb] - PPM_PULSE_LENGTH) * CLOCK_MULTIPLIER;
      calc_rest += PPM[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}

