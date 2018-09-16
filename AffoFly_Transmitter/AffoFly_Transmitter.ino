#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h>
#include <U8g2lib.h>
#include "printf.h"

//==== System Setting =============================================//
#define VERSION_NUMBER     "0.012"
//#define DEBUG_MODE  1
#define SHOW_RATE   1
//=======================================================================//

//==== Pin Definition ===================================================//
// PIN A4, A5 are used as I2C to connect LCD screen
// Pin 11, 12, 13 are used as SPI to connect NRF module
// Becareful: when use A6, A7 pin
//      A6, A7 are analog input only, also have no internal pullup.
//      You will need to add external pullup/pulldown resistor for them.
#define THROTTLE_PIN  A0
#define YAW_PIN       A1
#define PITCH_PIN     A2
#define ROLL_PIN      A3
#define V_BATTERY_PIN A6
#define AUX6_PIN      A7  //SWD
#define AUX1_PIN      2   //Arm - 2 states (bool)                                             //Menu: Save
#define AUX2_PIN      3   //Flight Mode - 3 states but could be more (byte: Acro/Angle/...)   //Menu: Up
#define AUX3_PIN      4   //Bomb Drop - 2 states (bool)                                       //Menu: Down
#define AUX4_PIN      5   //Beacon - 2 state (bool)                                           
#define AUX5_PIN      6   //
#define NRF_CE_PIN    7
#define NRF_CSN_PIN   8
#define BUZZER_PIN    9   //OUTPUT                                                            
#define MENU_PIN      10
//=======================================================================//

//==== Global Constant & Variable =======================================//
uint16_t BatteryVoltage = 0;
uint16_t LowBatteryVoltageThreshold = 3200; // mV

#ifdef SHOW_RATE
uint8_t SendRate = 0;
uint16_t LoopRate = 0;
uint8_t SendCount = 0;
uint16_t LoopCount = 0;
#endif

bool FunctionMode = false;

const uint8_t RADIO_UNIQUE_ID_EEPROM_ADDRESS = 0; //(2 bytes 0-1)
const uint16_t RADIO_UNIQUE_ID_LOWER_BOUNDARY = 1000;
const uint16_t RADIO_UNIQUE_ID_UPPER_BOUNDARY = 9999;
uint16_t RadioUniqueId = RADIO_UNIQUE_ID_LOWER_BOUNDARY;

const uint8_t RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS = 4;
const uint8_t RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY = 0; //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const uint8_t RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY = 3; //TODO: use index instead
int8_t RadioPaLevelIndex = 3;

const uint8_t CURRENT_RX_ID_EEPROM_ADDRESS = 5; //TODO: Implement Multiple RX support
const uint8_t CURRENT_RX_ID_LOWER_BOUNDARY = 1;
const uint8_t CURRENT_RX_ID_UPPER_BOUNDARY = 10;
uint8_t CurrentRxId = 1;

const uint8_t RX_CONFIG_EEPROM_START_ADDRESS = 100;
const uint8_t RX_CONFIG_ALLOCATED_BYTES = 20;
const uint8_t RX_TRIM_STEP_RANGE = 15;
const uint8_t RX_TRIM_STEP_WIDTH = 2;

const uint8_t RADIO_CHANNEL_LOWER_BOUNDARY = 100;
const uint8_t RADIO_CHANNEL_UPPER_BOUNDARY = 125;
const uint64_t RADIO_PIPE_OUT = 0xE8E8F0F0E1LL;
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

const uint8_t AUX1_VALUE_LOWER_BOUNDARY = 0;
const uint8_t AUX1_VALUE_UPPER_BOUNDARY = 1;
uint8_t Aux1Value = AUX1_VALUE_LOWER_BOUNDARY;
Bounce Aux1Bounce = Bounce();
bool Aux1Pressed = false;
const uint8_t AUX2_VALUE_LOWER_BOUNDARY = 0;
const uint8_t AUX2_VALUE_UPPER_BOUNDARY = 2;
uint8_t Aux2Value = AUX2_VALUE_LOWER_BOUNDARY;
Bounce Aux2Bounce = Bounce();
bool Aux2Pressed = false;
const uint8_t AUX3_VALUE_LOWER_BOUNDARY = 0;
const uint8_t AUX3_VALUE_UPPER_BOUNDARY = 1;
uint8_t Aux3Value = AUX3_VALUE_LOWER_BOUNDARY;
Bounce Aux3Bounce = Bounce();
bool Aux3Pressed = false;
const uint8_t AUX4_VALUE_LOWER_BOUNDARY = 0;
const uint8_t AUX4_VALUE_UPPER_BOUNDARY = 1;
uint8_t Aux4Value = AUX4_VALUE_LOWER_BOUNDARY;
Bounce Aux4Bounce = Bounce();
bool Aux4Pressed = false;
const uint8_t AUX5_VALUE_LOWER_BOUNDARY = 0;
const uint8_t AUX5_VALUE_UPPER_BOUNDARY = 3;
uint8_t Aux5Value = AUX5_VALUE_LOWER_BOUNDARY;
Bounce Aux5Bounce = Bounce();
bool Aux5Pressed = false;

Bounce MenuBounce = Bounce();
bool MenuPressed = false;

// NRF24L01+ module can send maximum 32 byte per package
struct MyData {
  uint16_t UniqueId;
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

const uint16_t BatteryVoltageReadInterval = 10000;
unsigned long BatteryVoltageReadLastMillis = 0;

const uint16_t FunctionWelcomeScreenShowDuration = 2000;

const uint16_t TimerRefreshInterval = 1000;
unsigned long TimerRefreshLastMillis = 0;
uint16_t Timer_Seconds = 0;

const uint16_t ScreenRefreshInterval = 200;
unsigned long ScreenRefreshLastMillis = 0;

bool BuzzerEnabled = true;
uint8_t BuzzerTimesToBeep = 0;
byte BuzzerState = LOW;
uint16_t BuzzerBeepInterval = 50;
unsigned long BuzzerBeepLastMillis = 0;

const uint8_t RadioSendInterval = 5;
unsigned long RadioSendLastMillis = 0;

char FlightMode[3][10] = {
  "ACRO",
  "ANGLE",
  "HORIZON"
};

// RxConfig data limit is 20 bytes
// We allocates 10 RxConfig in EEPROM from address 100 to 299
struct RxConfigData {
  uint8_t Id;
  uint8_t RadioChannel;
  int8_t ThrottleTrim;
  int8_t YawTrim;
  int8_t PitchTrim;
  int8_t RollTrim;
  char Name[10];
};
RxConfigData RxConfigs[10];

struct RadioPaLevelData {
  uint8_t PaValue;
  char PaName[5];
};

RadioPaLevelData RadioPaLevels[4] = {
  {RF24_PA_MIN, "MIN"},
  {RF24_PA_LOW, "LOW"},
  {RF24_PA_HIGH, "HIGH"},
  {RF24_PA_MAX, "MAX"}
};

char Functions[4][15] = {
  "Transmitter ID",
  "Radio PA Level",
  "RX Selection",
  "Wipe All Data"
};
uint8_t CurrentFunctionIndex = 0;
uint8_t SelectedFunctionIndex = 1;
uint8_t SelectedRxId = CurrentRxId;
uint8_t RxConfigurationIndex = 0;
uint8_t RxConfigurationNameIndex = 0;
char RxConfigurationName[10] = "";
uint8_t RxConfigurationChannel = 100;

//=======================================================================//

void setup() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont12_tr);
  readEeprom();
  pinMode(AUX1_PIN, INPUT_PULLUP);
  pinMode(AUX2_PIN, INPUT_PULLUP);
  pinMode(AUX3_PIN, INPUT_PULLUP);
  pinMode(AUX4_PIN, INPUT_PULLUP);
  pinMode(AUX5_PIN, INPUT_PULLUP);
  pinMode(MENU_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  Aux1Bounce.attach(AUX1_PIN);
  Aux2Bounce.attach(AUX2_PIN);
  Aux3Bounce.attach(AUX3_PIN);
  Aux4Bounce.attach(AUX4_PIN);
  Aux5Bounce.attach(AUX5_PIN);
  MenuBounce.attach(MENU_PIN);
  Aux1Bounce.interval(5);
  Aux2Bounce.interval(5);
  Aux3Bounce.interval(5);
  Aux4Bounce.interval(5);
  Aux5Bounce.interval(5);
  MenuBounce.interval(5);

  MenuBounce.update();
  if (MenuBounce.read() == LOW) {
    FunctionMode = true;
    BuzzerTimesToBeep = 3;
  } else {
    FunctionMode = false;
    BuzzerTimesToBeep = 1;
  }

  // Seed RxConfigs if it does not already exist
  if (EEPROM.read(RX_CONFIG_EEPROM_START_ADDRESS) == 0) {
    putRxConfigs();
  }

  //FunctionMode = true; //For testing only, remove after.

  if (FunctionMode) {
    drawFunctionWelcomeScreen();
    getRxConfigs();
#ifdef DEBUG_MODE
    for (uint8_t i = 0; i < 10; i++) {
      Serial.println(RxConfigs[i].Id);
    }
#endif
  } else {
    RxConfigData rxConfig = getRxConfig(CurrentRxId);
    radio.begin();
    radio.setPALevel(RadioPaLevels[RadioPaLevelIndex].PaValue);
    radio.setAutoAck(false);
    radio.setChannel(rxConfig.RadioChannel);
    radio.setDataRate(RF24_250KBPS);
    radio.openWritingPipe(RADIO_PIPE_OUT);
    radio.startListening();
    radio.stopListening();
#ifdef DEBUG_MODE
    printf_begin();
    radio.printDetails();
    Serial.print("UniqueId "); Serial.print(RadioUniqueId);
    Serial.print("    Channel "); Serial.println(rxConfig.RadioChannel);
#endif
  }
  resetData();
  readBatteryVoltage(10001); //Dummy value > BatteryVoltageReadInterval to force battery read once

  //delete after
  char testStr[3] = "";
  getRxIdStr(testStr, 1);
  Serial.println(testStr);
}

void loop() {
  unsigned long currentMillis = millis();
  if (FunctionMode) {
    if (currentMillis > FunctionWelcomeScreenShowDuration) {
      setFunctionValues();
      beepBuzzer(currentMillis);
      refreshFunctionOperationScreen(currentMillis);
    }
  } else {
    readBatteryVoltage(currentMillis);
    setTime(currentMillis);
    setButtonsValue(currentMillis);
    setControlValues();
    beepBuzzer(currentMillis);
    sendPayloadData(currentMillis);
    refreshControlScreen(currentMillis);
  }
#ifdef SHOW_RATE
  LoopCount++;
#endif
}

void readEeprom() {
  EEPROM.get(RADIO_UNIQUE_ID_EEPROM_ADDRESS, RadioUniqueId);
  if (RadioUniqueId < RADIO_UNIQUE_ID_LOWER_BOUNDARY || RadioUniqueId > RADIO_UNIQUE_ID_UPPER_BOUNDARY) {
    RadioUniqueId = RADIO_UNIQUE_ID_LOWER_BOUNDARY;
    EEPROM.write(RADIO_UNIQUE_ID_EEPROM_ADDRESS, RadioUniqueId);
  }

  RadioPaLevelIndex = EEPROM.read(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS);
  if (RadioPaLevelIndex < RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY || RadioPaLevelIndex > RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY) {
    RadioPaLevelIndex = RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY;
    EEPROM.write(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS, RadioPaLevelIndex);
  }

  CurrentRxId = EEPROM.read(CURRENT_RX_ID_EEPROM_ADDRESS);
  if (CurrentRxId < CURRENT_RX_ID_LOWER_BOUNDARY || CurrentRxId > CURRENT_RX_ID_UPPER_BOUNDARY) {
    CurrentRxId = CURRENT_RX_ID_LOWER_BOUNDARY;
    EEPROM.write(CURRENT_RX_ID_EEPROM_ADDRESS, CurrentRxId);
  }
}

void putRxConfigs() {
  for (int i = CURRENT_RX_ID_LOWER_BOUNDARY; i <= CURRENT_RX_ID_UPPER_BOUNDARY; i++) {
    uint16_t startAddress = (uint16_t)RX_CONFIG_ALLOCATED_BYTES * (i - 1) + RX_CONFIG_EEPROM_START_ADDRESS;
    RxConfigData rxConfig;
    rxConfig.Id = i;
    rxConfig.RadioChannel = 100;
    rxConfig.ThrottleTrim = 0;
    rxConfig.YawTrim = 0;
    rxConfig.PitchTrim = 0;
    rxConfig.RollTrim = 0;
    char rxName[6] = "RX ";
    char rxIdText[3] = "";
    itoa(i, rxIdText, 10);
    if (i < 10) {
      strcat(rxName, "0");
    }
    strcat(rxName, rxIdText);
    strcat(rxName, "    ");
    strcpy(rxConfig.Name, rxName);
    EEPROM.put(startAddress, rxConfig);
  }
}

void getRxConfigs() {
  for (int i = CURRENT_RX_ID_LOWER_BOUNDARY; i <= CURRENT_RX_ID_UPPER_BOUNDARY; i++) {
    uint16_t startAddress = (uint16_t)RX_CONFIG_ALLOCATED_BYTES * (i - 1) + RX_CONFIG_EEPROM_START_ADDRESS;
    RxConfigData rxConfig;
    EEPROM.get(startAddress, rxConfig);
    RxConfigs[i - 1] = rxConfig;
  }
}

RxConfigData getRxConfig(uint8_t rxId) {
  uint16_t address = (uint16_t)RX_CONFIG_ALLOCATED_BYTES * (rxId - 1) + RX_CONFIG_EEPROM_START_ADDRESS;
  RxConfigData rxConfig;
  EEPROM.get(address, rxConfig);
  return rxConfig;
}

void clearEeprom() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.update(i, 0);
  }
  softReset();
}

void setFunctionValues() {
  Aux1Bounce.update();
  Aux2Bounce.update();
  Aux3Bounce.update();
  Aux4Bounce.update();
  Aux5Bounce.update();
  MenuBounce.update();
  int pressedButtonIndex = -1;

  if (MenuBounce.read() == LOW) {
    if (!MenuPressed) {
      MenuPressed = true;
      pressedButtonIndex = 0;
      BuzzerTimesToBeep = 1;
    }
  } else {
    MenuPressed = false;
  }
  if (Aux1Bounce.read() == LOW) {
    if (!Aux1Pressed) {
      Aux1Pressed = true;
      pressedButtonIndex = 2;
      BuzzerTimesToBeep = 1;
    }
  } else {
    Aux1Pressed = false;
  }
  if (Aux4Bounce.read() == LOW) {
    if (!Aux4Pressed) {
      Aux4Pressed = true;
      pressedButtonIndex = 1;
      BuzzerTimesToBeep = 1;
    }
  } else {
    Aux4Pressed = false;
  }
  if (Aux5Bounce.read() == LOW) {
    if (!Aux5Pressed) {
      Aux5Pressed = true;
      pressedButtonIndex = 3;
      BuzzerTimesToBeep = 1;
    }
  } else {
    Aux5Pressed = false;
  }
  if (Aux3Bounce.read() == LOW) {
    if (!Aux3Pressed) {
      Aux3Pressed = true;
      pressedButtonIndex = 4;
      BuzzerTimesToBeep = 1;
    }
  } else {
    Aux3Pressed = false;
  }

  if (pressedButtonIndex > -1) {
    if (CurrentFunctionIndex == 0) {
      //Currently at Main Menu
      switch (pressedButtonIndex) {
        case 0: //Cancel
          break;
        case 1: //Confirm
          CurrentFunctionIndex = SelectedFunctionIndex;
          readEeprom();
          break;
        case 2: //Up
          SelectedFunctionIndex--;
          break;
        case 3: //Down
          SelectedFunctionIndex++;
          break;
      }
      uint8_t functionCount = sizeof(Functions) / sizeof(Functions[0]);
      if (SelectedFunctionIndex > functionCount) SelectedFunctionIndex = 1;
      if (SelectedFunctionIndex < 1) SelectedFunctionIndex = functionCount;
    }
    else if (CurrentFunctionIndex == 1) {
      //Currently at Transmitter Id function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
          EEPROM.put(RADIO_UNIQUE_ID_EEPROM_ADDRESS, RadioUniqueId);
          break;
        case 2: //Up
          RadioUniqueId++;
          break;
        case 3: //Down
          RadioUniqueId--;
          break;
          //case 4: //Move Position TODO: make it easier to change value by each digit
      }
      if (RadioUniqueId < RADIO_UNIQUE_ID_LOWER_BOUNDARY) RadioUniqueId = RADIO_UNIQUE_ID_UPPER_BOUNDARY;
      if (RadioUniqueId > RADIO_UNIQUE_ID_UPPER_BOUNDARY) RadioUniqueId = RADIO_UNIQUE_ID_LOWER_BOUNDARY;
    }
    else if (CurrentFunctionIndex == 2) {
      //Currently at Radio PA Level function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
          EEPROM.update(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS, RadioPaLevelIndex);
          break;
        case 2: //Up
          RadioPaLevelIndex++;
          break;
        case 3: //Down
          RadioPaLevelIndex--;
          break;
      }
      if (RadioPaLevelIndex < RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY) RadioPaLevelIndex = RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY;
      if (RadioPaLevelIndex > RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY) RadioPaLevelIndex = RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY;
    }
    else if (CurrentFunctionIndex == 3) {
      //Currently at RX Selection function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          SelectedRxId = CurrentRxId;
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
#ifdef DEBUG_MODE
          Serial.print("SelectedRxId: "); Serial.println(SelectedRxId);
#endif
          CurrentRxId = SelectedRxId;
          EEPROM.update(CURRENT_RX_ID_EEPROM_ADDRESS, CurrentRxId);
          strcpy(RxConfigurationName, RxConfigs[CurrentRxId - 1].Name);
          RxConfigurationChannel = RxConfigs[CurrentRxId - 1].RadioChannel;
          RxConfigurationIndex = 0;
          RxConfigurationNameIndex = 0;
          CurrentFunctionIndex = 10; // 10 for RX Configuration function
          break;
        case 2: //Up
          SelectedRxId--;
          break;
        case 3: //Down
          SelectedRxId++;
          break;
      }
      if (SelectedRxId < CURRENT_RX_ID_LOWER_BOUNDARY) SelectedRxId = CURRENT_RX_ID_UPPER_BOUNDARY;
      if (SelectedRxId > CURRENT_RX_ID_UPPER_BOUNDARY) SelectedRxId = CURRENT_RX_ID_LOWER_BOUNDARY;
    }
    else if (CurrentFunctionIndex == 4) {
      //Currently at Wipe All Data function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
          clearEeprom();
          break;
      }
    }
    else if (CurrentFunctionIndex == 10) {
      //Currently at RX Configuration function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
          strcpy(RxConfigs[CurrentRxId - 1].Name, RxConfigurationName);
          RxConfigs[CurrentRxId - 1].RadioChannel = RxConfigurationChannel;
          EEPROM.put(RX_CONFIG_EEPROM_START_ADDRESS + (CurrentRxId - 1) * RX_CONFIG_ALLOCATED_BYTES, RxConfigs[CurrentRxId - 1]);
          break;
        case 2: //Up
          switch (RxConfigurationIndex) {
            case 0: //RX Name
              RxConfigurationName[RxConfigurationNameIndex] -= 1; 
              if(RxConfigurationName[RxConfigurationNameIndex] < 32) RxConfigurationName[RxConfigurationNameIndex] = 126;
              break;
            case 1: //RX Channel
              RxConfigurationChannel++;
              if (RxConfigurationChannel > RADIO_CHANNEL_UPPER_BOUNDARY) RxConfigurationChannel = RADIO_CHANNEL_LOWER_BOUNDARY;
          }
          break;
        case 3: //Down
          switch (RxConfigurationIndex) {
            case 0: //RX Name
#ifdef DEBUG_MODE
  Serial.println(RxConfigurationName);
#endif
              RxConfigurationName[RxConfigurationNameIndex] += 1; 
              if(RxConfigurationName[RxConfigurationNameIndex] > 126) RxConfigurationName[RxConfigurationNameIndex] = 32;
#ifdef DEBUG_MODE
  Serial.println(RxConfigurationName);
#endif
              break;
            case 1: //RX Channel
              RxConfigurationChannel--;
              if (RxConfigurationChannel < RADIO_CHANNEL_LOWER_BOUNDARY) RxConfigurationChannel = RADIO_CHANNEL_UPPER_BOUNDARY;
          }
          break;
        case 4: //Move Position
          if (RxConfigurationIndex == 0) {
            RxConfigurationNameIndex++;
            if (RxConfigurationNameIndex > 8) { //TODO: Make it globle variable
              RxConfigurationNameIndex = 0;
              RxConfigurationIndex = 1;
            }
          } else {
            RxConfigurationIndex = 0;
          }
          break;
      }
    }
    else {
      //Other function
    }
  }
}

void setControlValues() {
  //TODO:
}

void setTime(unsigned long currentMillis) {
  if (currentMillis - TimerRefreshLastMillis >= TimerRefreshInterval) {
    TimerRefreshLastMillis = currentMillis;
    if (Aux1Value > 0)
      Timer_Seconds++;
    else
      Timer_Seconds = 0;
#ifdef SHOW_RATE
    SendRate = SendCount;
    LoopRate = LoopCount;
    SendCount = 0;
    LoopCount = 0;
#endif
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
}

uint16_t mapJoystickValues(uint16_t val, uint16_t lower, uint16_t middle, uint16_t upper, bool reverse) {
  val = constrain(val, lower, upper);
  if (val < middle) val = map(val, lower, middle, 1000, 1500);
  else val = map(val, middle, upper, 1500, 2000);
  return reverse ? 3000 - val : val;
}

void setButtonsValue(unsigned long currentMillis) {
  Aux1Bounce.update();
  Aux2Bounce.update();
  Aux3Bounce.update();
  Aux4Bounce.update();
  Aux5Bounce.update();
  MenuBounce.update();

  if (Aux1Bounce.read() == LOW) {
    if (!Aux1Pressed) {
      //For safty prevent arming when throttle is not off
      if (data.Throttle > 1000) {
        BuzzerTimesToBeep = 5;
        return;
      }
      Aux1Value++;
      if (Aux1Value > AUX1_VALUE_UPPER_BOUNDARY) {
        Aux1Value = AUX1_VALUE_LOWER_BOUNDARY;
      }
      Aux1Pressed = true;
      BuzzerTimesToBeep = Aux1Value + 1;
    }
  } else {
    Aux1Pressed = false;
  }

  if (Aux2Bounce.read() == LOW) {
    if (!Aux2Pressed) {
      Aux2Value++;
      if (Aux2Value > AUX2_VALUE_UPPER_BOUNDARY) {
        Aux2Value = AUX2_VALUE_LOWER_BOUNDARY;
      }
      Aux2Pressed = true;
      BuzzerTimesToBeep = Aux2Value + 1;
    }
  } else {
    Aux2Pressed = false;
  }

  if (Aux3Bounce.read() == LOW) {
    if (!Aux3Pressed) {
      Aux3Value++;
      if (Aux3Value > AUX3_VALUE_UPPER_BOUNDARY) {
        Aux3Value = AUX3_VALUE_LOWER_BOUNDARY;
      }
      Aux3Pressed = true;
      BuzzerTimesToBeep = Aux3Value + 1;
    }
  } else {
    Aux3Pressed = false;
  }

  if (Aux4Bounce.read() == LOW) {
    if (!Aux4Pressed) {
      Aux4Value++;
      if (Aux4Value > AUX4_VALUE_UPPER_BOUNDARY) {
        Aux4Value = AUX4_VALUE_LOWER_BOUNDARY;
      }
      Aux4Pressed = true;
      BuzzerTimesToBeep = Aux4Value + 1;
    }
  } else {
    Aux4Pressed = false;
  }

  if (Aux5Bounce.read() == LOW) {
    if (!Aux5Pressed) {
      Aux5Value++;
      if (Aux5Value > AUX5_VALUE_UPPER_BOUNDARY) {
        Aux5Value = AUX5_VALUE_LOWER_BOUNDARY;
      }
      Aux5Pressed = true;
      BuzzerTimesToBeep = Aux5Value + 1;
    }
  } else {
    Aux5Pressed = false;
  }

  if (MenuBounce.read() == LOW) {
    if (Aux1Value > 0) {
      // Any kinds of Armed state, prevent from disconnection
      BuzzerTimesToBeep = 10;
    } else {
      if (!MenuPressed) {
        MenuPressed = true;
        BuzzerTimesToBeep = 1;
      }
    }
  } else {
    MenuPressed = false;
  }
}

void readBatteryVoltage(unsigned long currentMillis) {
  if (currentMillis - BatteryVoltageReadLastMillis >= BatteryVoltageReadInterval) {
    BatteryVoltageReadLastMillis = currentMillis;
    uint8_t batteryVoltageSampleCount = 10; //Some value greater than 4 would make sense, keep it small though.
    uint16_t batteryVoltageValues[batteryVoltageSampleCount];
    for (uint8_t i = 0; i < batteryVoltageSampleCount; i++) {
      batteryVoltageValues[i] = analogRead(V_BATTERY_PIN);
    }
    sort(batteryVoltageValues, batteryVoltageSampleCount);
    uint16_t aggregatedValue = 0;
    for (uint8_t i = 2; i < batteryVoltageSampleCount - 2; i++) {
      aggregatedValue += batteryVoltageValues[i];
    }
    uint16_t averageValue = aggregatedValue / (batteryVoltageSampleCount - 4);
    // Here using 4.7k/10k voltage divider, adjust up 3%.
    uint8_t adjustPersentage = 3;
    BatteryVoltage = averageValue * 0.1007937 * (100 + adjustPersentage); // (147 / 47) * 33 / 1024)
#ifdef DEBUG_MODE
    Serial.print("Voltage: "); Serial.println(BatteryVoltage);
#endif
    if (BatteryVoltage < LowBatteryVoltageThreshold) {
      BuzzerTimesToBeep = 10;
    }
  }
}

void beepBuzzer(unsigned long currentMillis) {
  if (!BuzzerEnabled) return;
  if (BuzzerTimesToBeep > 0 || BuzzerState == HIGH) {
    if (currentMillis - BuzzerBeepLastMillis >= BuzzerBeepInterval) {
      BuzzerBeepLastMillis = currentMillis;
      if (BuzzerState == LOW) {
        BuzzerState = HIGH;
        BuzzerTimesToBeep--;
      } else {
        BuzzerState = LOW;
      }
      digitalWrite(BUZZER_PIN, BuzzerState);
    }
  }
}

void getPayloadData() {
  data.UniqueId = RadioUniqueId;
  //Get the middle value for Yaw, Pitch and Roll from Serial Monitor
  data.Throttle = mapJoystickValues(analogRead(THROTTLE_PIN), 0, 511, 1023, false );
  data.Yaw      = mapJoystickValues(analogRead(YAW_PIN),  0, 515, 1023, true );
  data.Pitch    = mapJoystickValues(analogRead(PITCH_PIN), 0, 513, 1023, false );
  data.Roll     = mapJoystickValues(analogRead(ROLL_PIN), 0, 504, 1023, true );
  data.Aux1     = map(Aux1Value, AUX1_VALUE_LOWER_BOUNDARY, AUX1_VALUE_UPPER_BOUNDARY, 1000, 2000);
  data.Aux2     = map(Aux2Value, AUX2_VALUE_LOWER_BOUNDARY, AUX2_VALUE_UPPER_BOUNDARY, 1000, 2000);
  data.Aux3     = map(Aux3Value, AUX3_VALUE_LOWER_BOUNDARY, AUX3_VALUE_UPPER_BOUNDARY, 1000, 2000);
  data.Aux4     = map(Aux4Value, AUX4_VALUE_LOWER_BOUNDARY, AUX4_VALUE_UPPER_BOUNDARY, 1000, 2000);
  data.Aux5     = map(Aux5Value, AUX5_VALUE_LOWER_BOUNDARY, AUX5_VALUE_UPPER_BOUNDARY, 1000, 2000);
  data.Aux6     = mapJoystickValues(analogRead(AUX6_PIN), 0, 511, 1023, false );
  //#ifdef DEBUG_MODE
  //  Serial.print("Throttle: "); Serial.print(analogRead(THROTTLE_PIN));   Serial.print("    ");
  //  Serial.print("Yaw: ");      Serial.print(analogRead(YAW_PIN));        Serial.print("    ");
  //  Serial.print("Pitch: ");    Serial.print(analogRead(PITCH_PIN));      Serial.print("    ");
  //  Serial.print("Roll: ");     Serial.print(analogRead(ROLL_PIN));       Serial.print("    ");
  //  Serial.print("Aux1: ");     Serial.print(Aux1Value);                  Serial.print("    ");
  //  Serial.print("Aux2: ");     Serial.print(Aux2Value);                  Serial.print("    ");
  //  Serial.print("Aux3: ");     Serial.print(Aux3Value);                  Serial.print("    ");
  //  Serial.print("Aux4: ");     Serial.print(Aux4Value);                  Serial.print("    ");
  //  Serial.print("Aux5: ");     Serial.print(Aux5Value);                  Serial.print("    ");
  //  Serial.print("Aux6: ");     Serial.print(analogRead(AUX6_PIN));       Serial.print("\n");
  //
  //  Serial.print("UniqueId: "); Serial.print(data.UniqueId);  Serial.print("    ");
  //  Serial.print("Throttle: "); Serial.print(data.Throttle);  Serial.print("    ");
  //  Serial.print("Yaw: ");      Serial.print(data.Yaw);       Serial.print("    ");
  //  Serial.print("Pitch: ");    Serial.print(data.Pitch);     Serial.print("    ");
  //  Serial.print("Roll: ");     Serial.print(data.Roll);      Serial.print("    ");
  //  Serial.print("Aux1: ");     Serial.print(data.Aux1);      Serial.print("    ");
  //  Serial.print("Aux2: ");     Serial.print(data.Aux2);      Serial.print("    ");
  //  Serial.print("Aux3: ");     Serial.print(data.Aux3);      Serial.print("    ");
  //  Serial.print("Aux4: ");     Serial.print(data.Aux4);      Serial.print("    ");
  //  Serial.print("Aux5: ");     Serial.print(data.Aux5);      Serial.print("    ");
  //  Serial.print("Aux6: ");     Serial.print(data.Aux6);      Serial.print("\n");
  //#endif
}

void sendPayloadData(unsigned long currentMillis) {
  if (currentMillis - RadioSendLastMillis >= RadioSendInterval) {
    RadioSendLastMillis = currentMillis;
    getPayloadData();
    radio.write(&data, sizeof(MyData));
#ifdef SHOW_RATE
    SendCount++;
#endif
  }
}

void drawFunctionWelcomeScreen() {
  u8g2.firstPage();
  do {
    char versionText[15] = "AffoFly V";
    strcat(versionText, VERSION_NUMBER);
    u8g2.drawStr(0, 10, versionText);
    u8g2.drawLine(0, 15, 128, 15);
  } while (u8g2.nextPage());
}

void refreshFunctionOperationScreen(unsigned long currentMillis) {
  if (currentMillis - ScreenRefreshLastMillis >= ScreenRefreshInterval) {
    ScreenRefreshLastMillis = currentMillis;
    u8g2.firstPage();
    do {
      switch (CurrentFunctionIndex) {
        case 0: //Main screen
          drawFunctionMainScreen();
          break;
        case 1: //Transmitter ID screen
          drawFunctionTransmitterIdScreen();
          break;
        case 2: //Radio PA Level screen
          drawFunctionRadioPaLevelScreen();
          break;
        case 3: //RX Selection screen
          drawFunctionRxSelectionScreen();
          break;
        case 4: //Wipe All Data screen
          drawFunctionWipeEepromScreen();
          break;
        case 10: //RX Configuration screen
          drawFunctionRxConfigurationScreen();
          break;
      }
    } while (u8g2.nextPage());
  }
}

void drawFunctionMainScreen() {
  u8g2.drawStr(0, 10, "Functions");
  u8g2.drawLine(0, 15, 128, 15);
  uint8_t functionCount = sizeof(Functions) / sizeof(Functions[0]);
  uint8_t menuItemStartX = 10;
  uint8_t menuItemStartY = 25;
  uint8_t lineHeight = 12;
  for (uint8_t i = 0; i < functionCount; i++) {
    u8g2.drawStr(menuItemStartX, menuItemStartY, Functions[i]);
    menuItemStartY += lineHeight;
  }
  uint8_t selectionBoxStartX = 1;
  uint8_t selectionBoxStartY = 19;
  uint8_t selectionBoxWidth = 4;
  uint8_t selectionBoxHeight = 4;
  selectionBoxStartY += lineHeight * (SelectedFunctionIndex - 1);
  u8g2.drawBox(selectionBoxStartX, selectionBoxStartY, selectionBoxWidth, selectionBoxHeight);
}

void drawFunctionTransmitterIdScreen() {
  u8g2.drawStr(0, 10, "Transmitter ID");
  u8g2.drawLine(0, 15, 128, 15);
  char strRadioUniqueId[5] = "";
  itoa(RadioUniqueId, strRadioUniqueId, 10);
  u8g2.drawStr(50, 40, strRadioUniqueId);
}

void drawFunctionRadioPaLevelScreen() {
  u8g2.drawStr(0, 10, "Radio PA Level");
  u8g2.drawLine(0, 15, 128, 15);
  u8g2.drawStr(50, 40, RadioPaLevels[RadioPaLevelIndex].PaName);
}

void drawFunctionRxSelectionScreen() {
  u8g2.drawStr(0, 10, "Select RX");
  char finalText[4] = "";
  char rxIdText[3] = "";
  itoa(CurrentRxId, rxIdText, 10);
  if (CurrentRxId < 10) {
    strcat(finalText, "0");
  }
  strcat(finalText, rxIdText);
  u8g2.drawStr(115, 10, finalText);
  u8g2.drawLine(0, 15, 128, 15);

  uint8_t pageSize = 4;
  uint8_t currentPage = SelectedRxId / pageSize;
  if (SelectedRxId % pageSize != 0 ) {
    currentPage += 1;
  }
  uint8_t startRxId = (currentPage - 1) * pageSize + 1;
  uint8_t menuItemStartX = 10;
  uint8_t menuItemStartY = 25;
  uint8_t lineHeight = 12;

  for (uint8_t i = 0; i < pageSize; i++) {
    if (startRxId + i > CURRENT_RX_ID_UPPER_BOUNDARY) break;
    char finalText[3] = "";
    char rxIdText[2] = "";
    uint8_t rxId = RxConfigs[startRxId + i - 1].Id;
    itoa(rxId, rxIdText, 10);
    if (rxId < 10) {
      strcat(finalText, "0");
    }
    strcat(finalText, rxIdText);
    strcat(finalText, "  ");
    strcat(finalText, RxConfigs[startRxId + i - 1].Name);
    u8g2.drawStr(menuItemStartX, menuItemStartY, finalText);
    menuItemStartY += lineHeight;
  }
  uint8_t selectionBoxStartX = 1;
  uint8_t selectionBoxStartY = 19;
  uint8_t selectionBoxWidth = 4;
  uint8_t selectionBoxHeight = 4;
  uint8_t selectionIndexOnPage = SelectedRxId % pageSize;
  if (selectionIndexOnPage == 0) selectionIndexOnPage = 4;
  selectionBoxStartY += lineHeight * (selectionIndexOnPage - 1);
  u8g2.drawBox(selectionBoxStartX, selectionBoxStartY, selectionBoxWidth, selectionBoxHeight);
}

void drawFunctionRxConfigurationScreen() {
  u8g2.drawStr(0, 10, "RX Configuration");
  char finalText[3] = "";
  char rxIdText[2] = "";
  itoa(CurrentRxId, rxIdText, 10);
  if (CurrentRxId < 10) {
    strcat(finalText, "0");
  }
  strcat(finalText, rxIdText);
  u8g2.drawStr(115, 10, finalText);
  u8g2.drawLine(0, 15, 128, 15);

  uint8_t menuItemStartY = 25;
  uint8_t lineHeight = 12;
  char rxNameText[17] = "Name: ";
  strcat(rxNameText, RxConfigurationName);
  u8g2.drawStr(0, menuItemStartY, rxNameText);

  char rxChannelText[13] = "Channel: ";
  char channelText[4] = "";
  itoa(RxConfigurationChannel, channelText, 10);
  strcat(rxChannelText, channelText);
  u8g2.drawStr(0, menuItemStartY + lineHeight, rxChannelText);

  if (RxConfigurationIndex == 0) {
    uint8_t firstLetterStartX = 36;
    uint8_t letterWidth = 5;
    uint8_t startX = firstLetterStartX + RxConfigurationNameIndex * letterWidth + RxConfigurationNameIndex;
    u8g2.drawLine(startX, menuItemStartY + 1, startX + letterWidth, menuItemStartY + 1);
  } else if (RxConfigurationIndex == 1) {
    uint8_t channelNumberStartX = 54;
    uint8_t channelNumberWidth = 17;
    u8g2.drawLine(channelNumberStartX, menuItemStartY + lineHeight + 1, channelNumberStartX + channelNumberWidth, menuItemStartY + lineHeight + 1);
  } else {

  }

}

void drawFunctionWipeEepromScreen() {
  u8g2.drawStr(0, 10, "Wipe All Data");
  u8g2.drawLine(0, 15, 128, 15);
  u8g2.drawStr(25, 40, "Are you sure?");
}

void refreshControlScreen(unsigned long currentMillis) {
  if (currentMillis - ScreenRefreshLastMillis >= ScreenRefreshInterval) {
    ScreenRefreshLastMillis = currentMillis;
    u8g2.firstPage();
    do {
      drawBattery(4200, 3200, BatteryVoltage, 0);
#ifdef SHOW_RATE
      drawRate();
#endif
      drawTimer();
      drawRx();
      drawFlightMode();
      drawTrim();
      drawAuxStatus(75, 28, "A3", data.Aux3);
      drawAuxStatus(75, 40, "A4", data.Aux4);
      drawAuxStatus(75, 52, "A5", data.Aux5);
      drawAuxStatus(75, 64, "A6", data.Aux6);
      //drawJoystick(0, 18, data.Yaw, data.Throttle);
      //drawJoystick(88, 18, data.Roll, data.Pitch);
    } while (u8g2.nextPage());
  }
}

void drawBattery(uint16_t maxVotage, uint16_t minVotage, uint16_t actualVotage, uint8_t startX) {
  u8g2.drawFrame(startX, 1, 35, 15);
  u8g2.drawLine(startX + 35, 5, startX + 35, 11);
  u8g2.drawLine(startX + 36, 6, startX + 36, 10);

  int8_t percentage = 0;
  if (actualVotage > minVotage) {
    percentage = (float)(actualVotage - minVotage) / (maxVotage - minVotage) * 100;
    while (percentage > 0) {
      startX += 3;
      u8g2.drawFrame(startX, 3, 2, 11);
      percentage -= 10;
    }
  }

  char finalText[6] = "";
  dtostrf((float)actualVotage / 1000, 6, 2, finalText);
  strcat(finalText, "v");
  u8g2.drawStr(30, 13, finalText);
}

void drawRate() {
  if (Aux1Value == 0) {
    char text[12] = "";
    char sendRateText[4] = "";
    char loopRateText[5] = "";
    itoa(SendRate, sendRateText, 10);
    itoa(LoopRate, loopRateText, 10);
    strcat(text, sendRateText);
    strcat(text, "|");
    strcat(text, loopRateText);
    u8g2.drawStr(80, 13, text);
  }
}

void drawTimer() {
  if (Aux1Value > 0) {
    u8g2.drawFrame(91, 1, 37, 15);
    char minutes_str[3];
    char seconds_str[3];
    char time_str[6] = "";
    uint8_t minute = (int)Timer_Seconds / 60;
    uint8_t second = Timer_Seconds % 60;
    itoa(minute, minutes_str, 10);
    itoa(second, seconds_str, 10);
    if (minute < 10) {
      strcat(time_str, "0");
    }
    strcat(time_str, minutes_str);
    strcat(time_str, ":");
    if (second < 10) {
      strcat(time_str, "0");
    }
    strcat(time_str, seconds_str);
    u8g2.drawStr(95, 13, time_str);
  }
}

void drawRx() {
  char finalText[7] = "RX: ";
  char rxIdText[2] = "";
  itoa(CurrentRxId, rxIdText, 10);
  if (CurrentRxId < 10) {
    strcat(finalText, "0");
  }
  strcat(finalText, rxIdText);
  u8g2.drawStr(0, 28, finalText);
}

void drawFlightMode() {
  u8g2.drawStr(0, 40, FlightMode[Aux2Value]);
}

void drawTrim() { //TODO: Get trim value from ROM
  u8g2.drawStr(0, 52, "Y");
  u8g2.drawLine(9, 48, 40, 48);
  u8g2.drawLine(25, 47, 25, 49); //Neutral mark
  uint8_t yawTrim = 2;
  u8g2.drawLine(25 + yawTrim, 46, 25 + yawTrim, 50);

  u8g2.drawStr(0, 64, "R");
  u8g2.drawLine(9, 60, 40, 60);
  u8g2.drawLine(25, 59, 25, 61); //Neutral mark
  uint8_t rollTrim = -3;
  u8g2.drawLine(25 + rollTrim, 58, 25 + rollTrim, 62);

  u8g2.drawStr(48, 64, "T");
  u8g2.drawLine(50, 20, 50, 51);
  u8g2.drawLine(49, 36, 51, 36); //Neutral mark
  uint8_t throttleTrim = 3;
  u8g2.drawLine(48, 36 + throttleTrim, 52, 36 + throttleTrim);

  u8g2.drawStr(60, 64, "P");
  u8g2.drawLine(62, 20, 62, 51);
  u8g2.drawLine(61, 36, 63, 36); //Neutral mark
  uint8_t pitchTrim = -2;
  u8g2.drawLine(60, 36 + pitchTrim, 64, 36 + pitchTrim);
}

void drawAuxStatus(uint8_t startX, uint8_t startY, char channel[2], uint16_t value) {
  u8g2.drawStr(startX, startY, channel);
  u8g2.drawFrame(startX + 13, startY - 8, 40, 8);
  uint16_t len = (value - 1000) / 25;
  //#ifdef DEBUG_MODE
  //  Serial.print(channel);  Serial.print(": "); Serial.print(value);  Serial.print("    "); Serial.println(len);
  //#endif
  u8g2.drawBox(startX + 13, startY - 8, len, 8);
}

void drawJoystick(byte startX, byte startY, byte valueX, byte valueY) {
  u8g2.drawFrame(startX, startY, 40, 40);
  float positionX = map(valueX, 0, 255, startX + 4, startX + 40 - 4);
  float positionY = map(valueY, 0, 255, startY + 40 - 4, startY + 4);
  if (positionX < startX + 4)
    positionX = startX + 4;
  if (positionX > startX + 40 - 5)
    positionX = startX + 40 - 5;
  if (positionY < startY + 4)
    positionY = startY + 4;
  if (positionY > startY + 40 - 5)
    positionY = startY + 40 - 5;
  u8g2.drawDisc(positionX, positionY, 3, U8G2_DRAW_ALL);
}

 void drawSignal(byte percentage, byte startX) {
   if (percentage > 0) {
     u8g2.drawLine(startX, 13, startX, 15); //20%
     u8g2.drawLine(startX + 1, 13, startX + 1, 15);
   }
   if (percentage > 20) {
     u8g2.drawLine(startX + 4, 10, startX + 4, 15);
     u8g2.drawLine(startX + 5, 10, startX + 5, 15);
   }
   if (percentage > 40) {
     u8g2.drawLine(startX + 8, 7, startX + 8, 15);
     u8g2.drawLine(startX + 9, 7, startX + 9, 15);
   }
   if (percentage > 60) {
     u8g2.drawLine(startX + 12, 4, startX + 12, 15);
     u8g2.drawLine(startX + 13, 4, startX + 13, 15);
   }
   if (percentage > 80) {
     u8g2.drawLine(startX + 16, 1, startX + 16, 15);
     u8g2.drawLine(startX + 17, 1, startX + 17, 15);
   }
 }

//TODO: investigate why this function does not work
void getRxIdStr(char* outStr, uint8_t rxId) {
  char finalText[3] = "";
  char rxIdText[3] = "";
  itoa(rxId, rxIdText, 10);
  if (rxId < 10) {
    strcat(finalText, "0");
  }
  strcat(finalText, rxIdText);
  strcpy(outStr, finalText);
}

void sort(int *a, int n) {
  for (int i = 1; i < n; ++i) {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

void softReset() {
  asm volatile ("  jmp 0");
}

