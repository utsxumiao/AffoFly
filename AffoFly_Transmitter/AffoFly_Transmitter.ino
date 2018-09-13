#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h>
#include <U8g2lib.h>
#include "printf.h"

//==== System Setting =============================================//
//#define DEBUG_MODE    1
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

bool FunctionMode = false;

const uint8_t RADIO_UNIQUE_ID_EEPROM_ADDRESS = 0; //TODO: use uint32_t (4 bytes) to provide a 8 digits UniqueId
const uint8_t RADIO_UNIQUE_ID_LOWER_BOUNDARY = 1;
const uint8_t RADIO_UNIQUE_ID_UPPER_BOUNDARY = 254;
uint8_t RadioUniqueId = RADIO_UNIQUE_ID_LOWER_BOUNDARY;
const uint8_t RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS = 1;
const uint8_t RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY = 0; //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const uint8_t RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY = 3; //TODO: use index instead
uint8_t RadioPaLevelIndex = 3;
// const uint8_t CURRENT_RECEIVER_ID_EEPROM_ADDRESS = 2; //TODO: Implement Multiple RX support
// const uint8_t CURRENT_RECEIVER_ID_LOWER_BOUNDARY = 1;
// const uint8_t CURRENT_RECEIVER_ID_UPPER_BOUNDARY = 4;
// uint8_t CurrentReceiverId = CURRENT_RECEIVER_ID_LOWER_BOUNDARY;
const uint8_t RADIO_CHANNEL_LOWER_BOUNDARY = 100;
const uint8_t RADIO_CHANNEL_UPPER_BOUNDARY = 125;
const uint8_t RADIO_CHANNEL_EEPROM_ADDRESS = 2;
uint8_t RadioChannel = RADIO_CHANNEL_LOWER_BOUNDARY;
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
  "Radio Channel",
  "Wipe All Data"
};
uint8_t CurrentFunctionIndex = 0;
uint8_t SelectedFunctionIndex = 1;

//=======================================================================//

#ifdef DEBUG_MODE
uint16_t sendCount = 0;
uint16_t loopCount = 0;
#endif

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

  //FunctionMode = true; //For testing only, remove after.

  if (FunctionMode) {
    drawFunctionWelcomeScreen();
  } else {
    radio.begin();
    radio.setPALevel(RadioPaLevels[RadioPaLevelIndex].PaValue);
    radio.setAutoAck(false);
    radio.setChannel(RadioChannel);
    radio.setDataRate(RF24_250KBPS);
    radio.openWritingPipe(RADIO_PIPE_OUT);
    radio.startListening();
    radio.stopListening();
#ifdef DEBUG_MODE
    printf_begin();
    radio.printDetails();
    Serial.print("UniqueId "); Serial.print(RadioUniqueId);
    Serial.print("    Channel "); Serial.println(RadioChannel);
#endif
  }
  resetData();
  readBatteryVoltage(10001); //Dummy value > BatteryVoltageReadInterval to force battery read once
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
#ifdef DEBUG_MODE
  loopCount++;
#endif
}

void readEeprom() {
  RadioUniqueId = EEPROM.read(RADIO_UNIQUE_ID_EEPROM_ADDRESS);
  if (RadioUniqueId < RADIO_UNIQUE_ID_LOWER_BOUNDARY || RadioUniqueId > RADIO_UNIQUE_ID_UPPER_BOUNDARY) {
    RadioUniqueId = RADIO_UNIQUE_ID_LOWER_BOUNDARY;
    EEPROM.write(RADIO_UNIQUE_ID_EEPROM_ADDRESS, RadioUniqueId);
  }

  RadioPaLevelIndex = EEPROM.read(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS);
  if (RadioPaLevelIndex < RADIO_PA_LEVEL_INDEX_LOWER_BOUNDARY || RadioPaLevelIndex > RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY) {
    RadioPaLevelIndex = RADIO_PA_LEVEL_INDEX_UPPER_BOUNDARY;
    EEPROM.write(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS, RadioPaLevelIndex);
  }

  RadioChannel = EEPROM.read(RADIO_CHANNEL_EEPROM_ADDRESS);
  if (RadioChannel < RADIO_CHANNEL_LOWER_BOUNDARY || RadioChannel > RADIO_CHANNEL_UPPER_BOUNDARY) {
    RadioChannel = RADIO_CHANNEL_LOWER_BOUNDARY;
    EEPROM.write(RADIO_CHANNEL_EEPROM_ADDRESS, RadioChannel);
  }
}

void clearEeprom() {
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
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
          EEPROM.write(RADIO_UNIQUE_ID_EEPROM_ADDRESS, RadioUniqueId);
          break;
        case 2: //Up
          RadioUniqueId++;
          break;
        case 3: //Down
          RadioUniqueId--;
          break;
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
          EEPROM.write(RADIO_PA_LEVEL_INDEX_EEPROM_ADDRESS, RadioPaLevelIndex);
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
      //Currently at Radio Channel function
      switch (pressedButtonIndex) {
        case 0: //Cancel
          CurrentFunctionIndex = 0;
          break;
        case 1: //Confirm
          EEPROM.write(RADIO_CHANNEL_EEPROM_ADDRESS, RadioChannel);
          break;
        case 2: //Up
          RadioChannel++;
          break;
        case 3: //Down
          RadioChannel--;
          break;
      }
      if (RadioChannel < RADIO_CHANNEL_LOWER_BOUNDARY) RadioChannel = RADIO_CHANNEL_UPPER_BOUNDARY;
      if (RadioChannel > RADIO_CHANNEL_UPPER_BOUNDARY) RadioChannel = RADIO_CHANNEL_LOWER_BOUNDARY;
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
#ifdef DEBUG_MODE
    Serial.print(sendCount);
    Serial.print("    ");
    Serial.println(loopCount);
    sendCount = 0;
    loopCount = 0;
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
      if(data.Throttle > 1000) {
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
#ifdef DEBUG_MODE
    sendCount++;
#endif
  }
}

void drawFunctionWelcomeScreen() {
  u8g2.firstPage();
  do {
    u8g2.drawLine(0, 0, 127, 0);
    u8g2.drawLine(0, 3, 127, 3);
    u8g2.drawLine(0, 6, 127, 6);
    u8g2.drawLine(0, 9, 127, 9);
    u8g2.drawLine(0, 12, 127, 12);
    u8g2.drawLine(0, 15, 127, 15);

    u8g2.drawStr(20, 50, "AffoFly V0.012");
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
        case 3: //Radio Channel screen
          drawFunctionRadioChannelScreen();
          break;
        case 4: //Wipe All Data screen
          drawFunctionWipeEepromScreen();
          break;
      }
    } while (u8g2.nextPage());
  }
}

void drawFunctionMainScreen() {
  u8g2.drawStr(0, 10, "Functions");
  u8g2.drawLine(0, 15, 128, 15);
  byte functionCount = sizeof(Functions) / sizeof(Functions[0]);
  byte menuItemStartX = 10;
  byte menuItemStartY = 25;
  byte lineHight = 12;
  for (int i = 0; i < functionCount; i++) {
    u8g2.drawStr(menuItemStartX, menuItemStartY, Functions[i]);
    menuItemStartY += lineHight;
  }
  byte selectionBoxStartX = 1;
  byte selectionBoxStartY = 19;
  byte selectionBoxWidth = 4;
  byte selectionBoxHeight = 4;
  selectionBoxStartY += lineHight * (SelectedFunctionIndex - 1);
  u8g2.drawBox(selectionBoxStartX, selectionBoxStartY, selectionBoxWidth, selectionBoxHeight);
}

void drawFunctionTransmitterIdScreen() {
  u8g2.drawStr(0, 10, "Transmitter ID");
  u8g2.drawLine(0, 15, 128, 15);
  char strRadioUniqueId[3];
  u8g2.drawStr(60, 40, itoa(RadioUniqueId, strRadioUniqueId, 10));
}

void drawFunctionRadioPaLevelScreen() {
  u8g2.drawStr(0, 10, "Radio PA Level");
  u8g2.drawLine(0, 15, 128, 15);
  u8g2.drawStr(50, 40, RadioPaLevels[RadioPaLevelIndex].PaName);
}

void drawFunctionRadioChannelScreen() {
  u8g2.drawStr(0, 10, "Radio Channel");
  u8g2.drawLine(0, 15, 128, 15);
  char strRadioChannel[3];
  u8g2.drawStr(50, 40, itoa(RadioChannel, strRadioChannel, 10));
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
      drawTimer();
      drawReceiver();
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

void drawReceiver() {
 char text[7] = "RX: ";
 strcat(text, "01");
 u8g2.drawStr(0, 28, text);
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

//void drawJoystick(byte startX, byte startY, byte valueX, byte valueY) {
//  u8g2.drawFrame(startX, startY, 40, 40);
//  float positionX = map(valueX, 0, 255, startX + 4, startX + 40 - 4);
//  float positionY = map(valueY, 0, 255, startY + 40 - 4, startY + 4);
//  if (positionX < startX + 4)
//    positionX = startX + 4;
//  if (positionX > startX + 40 - 5)
//    positionX = startX + 40 - 5;
//  if (positionY < startY + 4)
//    positionY = startY + 4;
//  if (positionY > startY + 40 - 5)
//    positionY = startY + 40 - 5;
//  u8g2.drawDisc(positionX, positionY, 3, U8G2_DRAW_ALL);
//}

// void drawSignal(byte percentage, byte startX) {
//   if (percentage > 0) {
//     u8g2.drawLine(startX, 13, startX, 15); //20%
//     u8g2.drawLine(startX + 1, 13, startX + 1, 15);
//   }
//   if (percentage > 20) {
//     u8g2.drawLine(startX + 4, 10, startX + 4, 15);
//     u8g2.drawLine(startX + 5, 10, startX + 5, 15);
//   }
//   if (percentage > 40) {
//     u8g2.drawLine(startX + 8, 7, startX + 8, 15);
//     u8g2.drawLine(startX + 9, 7, startX + 9, 15);
//   }
//   if (percentage > 60) {
//     u8g2.drawLine(startX + 12, 4, startX + 12, 15);
//     u8g2.drawLine(startX + 13, 4, startX + 13, 15);
//   }
//   if (percentage > 80) {
//     u8g2.drawLine(startX + 16, 1, startX + 16, 15);
//     u8g2.drawLine(startX + 17, 1, startX + 17, 15);
//   }
// }

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

